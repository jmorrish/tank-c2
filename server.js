const express = require('express');
const session = require('express-session');
const helmet = require('helmet');
const rateLimit = require('express-rate-limit');
const { WebSocketServer } = require('ws');
const net = require('net');
const path = require('path');
const http = require('http');
const fs = require('fs');
const crypto = require('crypto');
const Anthropic = require('@anthropic-ai/sdk');

const anthropic = process.env.ANTHROPIC_API_KEY
    ? new Anthropic({ apiKey: process.env.ANTHROPIC_API_KEY })
    : null;
if (!anthropic) console.warn('[learn] ANTHROPIC_API_KEY not set — AI descriptions disabled');

const JETSON_HOST  = process.env.JETSON_HOST  || '100.91.47.30';
const JETSON_PORT  = parseInt(process.env.JETSON_PORT  || '9999');
const MJPEG_PORT   = parseInt(process.env.MJPEG_PORT   || '8080');
const WEB_PORT     = parseInt(process.env.PORT         || '3000');
const MISSIONS_DIR = path.join(__dirname, 'missions');
const AUTH_PASSWORD = process.env.AUTH_PASSWORD;
if (!AUTH_PASSWORD) { console.error('FATAL: AUTH_PASSWORD env var not set'); process.exit(1); }
const SESSION_SECRET = process.env.SESSION_SECRET || (() => {
    console.warn('[WARN] SESSION_SECRET not set — sessions invalidated on restart. Set SESSION_SECRET env var.');
    return crypto.randomBytes(32).toString('hex');
})();

if (!fs.existsSync(MISSIONS_DIR)) fs.mkdirSync(MISSIONS_DIR, { recursive: true });

const app    = express();
const server = http.createServer(app);
const wss    = new WebSocketServer({ noServer: true });

app.set('trust proxy', 1); // trust nginx — required for rate limiter IP detection

app.use(helmet({
    contentSecurityPolicy: {
        directives: {
            defaultSrc:     ["'self'"],
            scriptSrc:      ["'self'", "'unsafe-inline'", "'wasm-unsafe-eval'", "https://www.gstatic.com"],
            styleSrc:       ["'self'", "'unsafe-inline'"],
            imgSrc:         ["'self'", "data:", "blob:",
                             "https://*.tile.openstreetmap.org",   // Leaflet OSM tiles
                             "https://server.arcgisonline.com"],   // Leaflet satellite tiles
            connectSrc:     ["'self'", "wss:", "blob:",
                             "https://www.gstatic.com",            // Draco WASM fetch
                             "https://ipapi.co"],                  // IP geolocation fallback
            mediaSrc:       ["'self'", "blob:"],
            workerSrc:      ["'self'", "blob:", "https://www.gstatic.com"],
            objectSrc:      ["'none'"],
            frameAncestors: ["'self'"],   // allow same-origin iframe (docs tab); blocks external embedding
            formAction:     ["'self'"],
            baseUri:        ["'self'"],
            scriptSrcAttr:  ["'unsafe-inline'"],  // allow onclick/onload attrs
        }
    },
    crossOriginEmbedderPolicy: false,  // model-viewer requires this disabled
    permissionsPolicy: false,          // don't block geolocation (map uses navigator.geolocation)
}));

// ── Sessions ──────────────────────────────────────────────────────────────────
app.use(session({
    secret: SESSION_SECRET,
    resave: false,
    saveUninitialized: false,
    cookie: { httpOnly: true, sameSite: 'strict', secure: true, maxAge: 4 * 60 * 60 * 1000 }
}));

// Short-lived tokens for WebSocket auth (browser requests one via HTTP then
// uses it in the WS handshake URL, so the WS upgrade can be authenticated
// without cookies — which aren't reliably sent with WS upgrades cross-origin).
const wsTokens = new Map(); // token → expiry timestamp
function issueWsToken() {
    const tok = crypto.randomBytes(16).toString('hex');
    wsTokens.set(tok, Date.now() + 30_000); // valid 30 s
    return tok;
}
function validateWsToken(tok) {
    const exp = wsTokens.get(tok);
    if (!exp) return false;
    wsTokens.delete(tok);
    return Date.now() < exp;
}

// ── Auth middleware ───────────────────────────────────────────────────────────
function requireAuth(req, res, next) {
    if (req.session && req.session.authed) return next();
    if (req.path === '/login' || req.path === '/login.html') return next();
    res.redirect('/login');
}

app.use(express.json({ limit: '1mb' }));
app.use(express.urlencoded({ extended: false }));

// ── Rate limiters ─────────────────────────────────────────────────────────────
const loginLimiter = rateLimit({
    windowMs: 15 * 60 * 1000,  // 15-minute window
    max: 10,                    // 10 attempts per IP per window
    standardHeaders: true,
    legacyHeaders: false,
    skipSuccessfulRequests: true,
    handler: (_req, res) => res.redirect('/login?err=limit'),
});

// Login / logout routes (before static middleware so they aren't blocked)
app.get('/login', (_req, res) => res.sendFile(path.join(__dirname, 'public', 'login.html')));

app.post('/login', loginLimiter, (req, res) => {
    if (req.body.password === AUTH_PASSWORD) {
        req.session.authed = true;
        return res.redirect('/');
    }
    res.redirect('/login?err=1');
});

app.post('/logout', (req, res) => {
    req.session.destroy(() => res.redirect('/login'));
});

// Issue a WS token for authenticated sessions
app.get('/api/ws-token', requireAuth, (_req, res) => {
    res.json({ token: issueWsToken() });
});

// Protect all other routes
app.use(requireAuth);
app.use('/api/', rateLimit({ windowMs: 60_000, max: 120, standardHeaders: true, legacyHeaders: false }));
app.use(express.static(path.join(__dirname, 'public')));

// ── MJPEG relay (one upstream connection → fan-out to all viewers) ───────────
// Keeps a single persistent connection to mjpeg_bridge on the Jetson.
// All browser /stream clients receive frames from this one connection so
// Jetson upload stays constant regardless of viewer count.
// /snapshot serves the latest JPEG for Safari/iOS which doesn't support MJPEG.
const BOUNDARY      = 'tankframe';
const streamClients = new Set();
let   latestFrame   = null;    // Buffer of the most recent JPEG
let   relayOnline   = false;   // true only while upstream connection is active

function dropStreamClients() {
    // Close all waiting /stream connections so browsers get an error and show NO SIGNAL
    for (const res of streamClients) { try { res.end(); } catch (_) {} }
    streamClients.clear();
}

function pushFrame(jpeg) {
    latestFrame = jpeg;
    const header = `--${BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpeg.length}\r\n\r\n`;
    for (const res of streamClients) {
        try { res.write(header); res.write(jpeg); res.write('\r\n'); }
        catch (_) { streamClients.delete(res); }
    }
}

function startMjpegRelay() {
    let buf = Buffer.alloc(0);
    const req = http.get(
        `http://${JETSON_HOST}:${MJPEG_PORT}/stream`,
        { timeout: 5000 },
        (upstream) => {
            relayOnline = true;
            upstream.on('data', chunk => {
                buf = Buffer.concat([buf, chunk]);
                while (true) {
                    const bStart = buf.indexOf(`--${BOUNDARY}\r\n`);
                    if (bStart === -1) { buf = Buffer.alloc(0); break; }
                    const hEnd = buf.indexOf('\r\n\r\n', bStart);
                    if (hEnd === -1) break;
                    const header   = buf.slice(bStart, hEnd).toString();
                    const lenMatch = header.match(/Content-Length:\s*(\d+)/i);
                    if (!lenMatch) { buf = buf.slice(bStart + 1); continue; }
                    const frameLen   = parseInt(lenMatch[1]);
                    const frameStart = hEnd + 4;
                    const frameEnd   = frameStart + frameLen;
                    if (buf.length < frameEnd) break;
                    pushFrame(buf.slice(frameStart, frameEnd));
                    buf = buf.slice(frameEnd);
                }
            });
            const done = () => {
                relayOnline = false; latestFrame = null;
                dropStreamClients();   // force browser onerror → NO SIGNAL
                setTimeout(startMjpegRelay, 3000);
            };
            upstream.on('end',   done);
            upstream.on('error', done);
        }
    );
    req.on('timeout', () => { req.destroy(); });
    req.on('error',   () => {
        relayOnline = false; latestFrame = null;
        dropStreamClients();
        setTimeout(startMjpegRelay, 3000);
    });
}
startMjpegRelay();

// ── Depth stream relay (stereo_depth_zmq → mjpeg_bridge /depth_stream) ───────
const depthStreamClients = new Set();
let   latestDepthFrame   = null;
let   depthRelayOnline   = false;

function dropDepthStreamClients() {
    for (const res of depthStreamClients) { try { res.end(); } catch (_) {} }
    depthStreamClients.clear();
}
function pushDepthFrame(jpeg) {
    latestDepthFrame = jpeg;
    const header = `--${BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpeg.length}\r\n\r\n`;
    for (const res of depthStreamClients) {
        try { res.write(header); res.write(jpeg); res.write('\r\n'); }
        catch (_) { depthStreamClients.delete(res); }
    }
}
function startDepthRelay() {
    let buf = Buffer.alloc(0);
    const req = http.get(
        `http://${JETSON_HOST}:${MJPEG_PORT}/depth_stream`,
        { timeout: 8000 },
        (upstream) => {
            // Clear idle timeout once connected — stream may be silent when stereo is off
            upstream.socket && upstream.socket.setTimeout(0);
            depthRelayOnline = true;
            upstream.on('data', chunk => {
                buf = Buffer.concat([buf, chunk]);
                while (true) {
                    const hEnd = buf.indexOf('\r\n\r\n');
                    if (hEnd < 0) break;
                    const header   = buf.slice(0, hEnd).toString();
                    const lenMatch = header.match(/Content-Length:\s*(\d+)/i);
                    if (!lenMatch) { buf = buf.slice(hEnd + 4); break; }
                    const frameLen   = parseInt(lenMatch[1]);
                    const frameStart = hEnd + 4;
                    const frameEnd   = frameStart + frameLen;
                    if (buf.length < frameEnd) break;
                    pushDepthFrame(buf.slice(frameStart, frameEnd));
                    buf = buf.slice(frameEnd);
                }
            });
            const done = () => {
                depthRelayOnline = false; latestDepthFrame = null;
                dropDepthStreamClients();
                setTimeout(startDepthRelay, 3000);
            };
            upstream.on('end',   done);
            upstream.on('error', done);
        }
    );
    req.on('timeout', () => req.destroy());
    req.on('error',   () => {
        depthRelayOnline = false; latestDepthFrame = null;
        dropDepthStreamClients();
        setTimeout(startDepthRelay, 3000);
    });
}
startDepthRelay();

app.get('/depth_stream', (req, res) => {
    if (!depthRelayOnline) return res.status(503).end();
    res.writeHead(200, {
        'Content-Type': `multipart/x-mixed-replace; boundary=${BOUNDARY}`,
        'Cache-Control': 'no-cache, no-store',
        'Connection':    'keep-alive',
    });
    if (latestDepthFrame) {
        res.write(`--${BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: ${latestDepthFrame.length}\r\n\r\n`);
        res.write(latestDepthFrame);
        res.write('\r\n');
    }
    depthStreamClients.add(res);
    req.on('close', () => depthStreamClients.delete(res));
});

app.get('/depth_snapshot', (req, res) => {
    if (!latestDepthFrame) return res.status(503).end();
    res.writeHead(200, { 'Content-Type': 'image/jpeg', 'Cache-Control': 'no-store' });
    res.end(latestDepthFrame);
});

// ── Thermal stream relay (RTSP thermal → mjpeg_bridge /thermal_stream) ────────
const thermalStreamClients = new Set();
let   latestThermalFrame   = null;
let   thermalRelayOnline   = false;

function dropThermalStreamClients() {
    for (const res of thermalStreamClients) { try { res.end(); } catch (_) {} }
    thermalStreamClients.clear();
}
function pushThermalFrame(jpeg) {
    latestThermalFrame = jpeg;
    const header = `--${BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpeg.length}\r\n\r\n`;
    for (const res of thermalStreamClients) {
        try { res.write(header); res.write(jpeg); res.write('\r\n'); }
        catch (_) { thermalStreamClients.delete(res); }
    }
}
function startThermalRelay() {
    let buf = Buffer.alloc(0);
    const req = http.get(
        `http://${JETSON_HOST}:${MJPEG_PORT}/thermal_stream`,
        { timeout: 8000 },
        (upstream) => {
            upstream.socket && upstream.socket.setTimeout(0);
            thermalRelayOnline = true;
            upstream.on('data', chunk => {
                buf = Buffer.concat([buf, chunk]);
                while (true) {
                    const hEnd = buf.indexOf('\r\n\r\n');
                    if (hEnd < 0) break;
                    const header   = buf.slice(0, hEnd).toString();
                    const lenMatch = header.match(/Content-Length:\s*(\d+)/i);
                    if (!lenMatch) { buf = buf.slice(hEnd + 4); break; }
                    const frameLen   = parseInt(lenMatch[1]);
                    const frameStart = hEnd + 4;
                    const frameEnd   = frameStart + frameLen;
                    if (buf.length < frameEnd) break;
                    pushThermalFrame(buf.slice(frameStart, frameEnd));
                    buf = buf.slice(frameEnd);
                }
            });
            const done = () => {
                thermalRelayOnline = false; latestThermalFrame = null;
                dropThermalStreamClients();
                setTimeout(startThermalRelay, 3000);
            };
            upstream.on('end',   done);
            upstream.on('error', done);
        }
    );
    req.on('timeout', () => req.destroy());
    req.on('error',   () => {
        thermalRelayOnline = false; latestThermalFrame = null;
        dropThermalStreamClients();
        setTimeout(startThermalRelay, 3000);
    });
}
startThermalRelay();

app.get('/thermal_stream', (req, res) => {
    if (!thermalRelayOnline) return res.status(503).end();
    res.writeHead(200, {
        'Content-Type': `multipart/x-mixed-replace; boundary=${BOUNDARY}`,
        'Cache-Control': 'no-cache, no-store',
        'Connection':    'keep-alive',
    });
    if (latestThermalFrame) {
        res.write(`--${BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: ${latestThermalFrame.length}\r\n\r\n`);
        res.write(latestThermalFrame);
        res.write('\r\n');
    }
    thermalStreamClients.add(res);
    req.on('close', () => thermalStreamClients.delete(res));
});

app.get('/thermal_snapshot', (req, res) => {
    if (!latestThermalFrame) return res.status(503).end();
    res.writeHead(200, { 'Content-Type': 'image/jpeg', 'Cache-Control': 'no-store' });
    res.end(latestThermalFrame);
});

// MJPEG stream for Chrome/Firefox/Android — returns 503 immediately if offline
app.get('/stream', (req, res) => {
    if (!relayOnline) return res.status(503).end();
    res.writeHead(200, {
        'Content-Type': `multipart/x-mixed-replace; boundary=${BOUNDARY}`,
        'Cache-Control': 'no-cache, no-store',
        'Connection':    'keep-alive',
    });
    if (latestFrame) {
        res.write(`--${BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: ${latestFrame.length}\r\n\r\n`);
        res.write(latestFrame);
        res.write('\r\n');
    }
    streamClients.add(res);
    req.on('close', () => streamClients.delete(res));
});

// ── Target gallery proxies → Jetson :8080 ────────────────────────────────────
app.get('/api/targets', (_req, res) => {
    http.get(`http://${JETSON_HOST}:${MJPEG_PORT}/targets`, { timeout: 3000 }, (upstream) => {
        res.setHeader('Content-Type', 'application/json');
        upstream.pipe(res);
    }).on('error', () => res.status(503).json([]));
});

app.get('/api/targets/:id/thumb', (req, res) => {
    const id = parseInt(req.params.id);
    if (isNaN(id) || id < 1) return res.status(400).end();
    http.get(`http://${JETSON_HOST}:${MJPEG_PORT}/targets/${id}/thumb`,
        { timeout: 3000 }, (upstream) => {
        res.setHeader('Content-Type', 'image/jpeg');
        upstream.pipe(res);
    }).on('error', () => res.status(404).end());
});

app.put('/api/targets/:id/name', (req, res) => {
    const id = parseInt(req.params.id);
    if (isNaN(id) || id < 1) return res.status(400).end();
    const body = JSON.stringify(req.body);
    const options = {
        hostname: JETSON_HOST, port: MJPEG_PORT,
        path: `/targets/${id}/name`, method: 'POST',
        headers: { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(body) },
        timeout: 3000,
    };
    const proxy = http.request(options, (upstream) => { upstream.pipe(res); });
    proxy.on('error', () => res.status(503).json({ ok: false }));
    proxy.write(body);
    proxy.end();
});

// Single JPEG snapshot for Safari/iOS (polled by JS at ~10 fps)
app.get('/snapshot', (req, res) => {
    if (!latestFrame) return res.status(503).end();
    res.writeHead(200, { 'Content-Type': 'image/jpeg', 'Cache-Control': 'no-store' });
    res.end(latestFrame);
});

// ── Jetson TCP bridge ─────────────────────────────────────────────────────────
let jetsonSocket    = null;
let jetsonBuffer    = '';
let lastStatus      = null;
let lastSlamMap     = null;
const DESCRIPTIONS_FILE = path.join(__dirname, 'descriptions.json');
const targetDescriptions = new Map(); // person_id → description string
// Load persisted descriptions from disk
try {
    const saved = JSON.parse(fs.readFileSync(DESCRIPTIONS_FILE, 'utf8'));
    for (const [k, v] of Object.entries(saved)) targetDescriptions.set(Number(k), v);
    console.log(`[learn] Loaded ${targetDescriptions.size} saved description(s)`);
} catch (_) { /* first run or missing file — fine */ }
let jetsonOnline    = false;
let jetsonReconnecting = false;
const CONNECT_TIMEOUT_MS = 8000;
const RECONNECT_DELAY_MS = 3000;
const clients = new Set();

// Queue for critical commands that must be delivered even after a reconnect.
// Stored as an array so multiple commands (e.g. abort then stop_follow) are not
// lost.  Capped at 5 to prevent unbounded growth.
const PENDING_CRITICAL_MAX = 5;
let pendingCritical = [];  // [{ cmd, broadcastOnDeliver }, ...]

function broadcast(msg) {
    const str = JSON.stringify(msg);
    for (const ws of clients) if (ws.readyState === 1) ws.send(str);
}

function sendToJetson(cmd) {
    if (jetsonSocket && !jetsonSocket.destroyed && jetsonOnline) {
        jetsonSocket.write(cmd + '\n');
        return true;
    }
    return false;
}

function scheduleReconnect() {
    if (jetsonReconnecting) return;
    jetsonReconnecting = true;
    setTimeout(() => { jetsonReconnecting = false; connectJetson(); }, RECONNECT_DELAY_MS);
}

function connectJetson() {
    console.log(`[jetson] Connecting to ${JETSON_HOST}:${JETSON_PORT}...`);
    jetsonBuffer = '';
    jetsonOnline = false;

    const sock = new net.Socket();

    // Bail out if the connect handshake hangs
    const connectTimer = setTimeout(() => {
        console.warn('[jetson] Connect timeout — retrying...');
        sock.destroy();
    }, CONNECT_TIMEOUT_MS);

    sock.connect(JETSON_PORT, JETSON_HOST, () => {
        clearTimeout(connectTimer);
        console.log('[jetson] Connected');
        jetsonSocket = sock;
        jetsonOnline = true;
        sock.setKeepAlive(true, 5000);
        broadcast({ type: 'connection', status: 'connected' });
        // Deliver any critical commands that were issued while offline
        if (pendingCritical.length) {
            const queue = pendingCritical.splice(0);
            for (const { cmd, broadcastOnDeliver } of queue) {
                sock.write(cmd + '\n');
                console.log(`[jetson] Delivered queued critical command: ${cmd}`);
                if (broadcastOnDeliver) broadcast(broadcastOnDeliver);
            }
        }
    });

    sock.on('data', (data) => {
        jetsonBuffer += data.toString();
        let pos;
        while ((pos = jetsonBuffer.indexOf('\n')) !== -1) {
            const line = jetsonBuffer.slice(0, pos).trim();
            jetsonBuffer = jetsonBuffer.slice(pos + 1);
            if (!line) continue;
            try {
                const parsed = JSON.parse(line);
                if (parsed.type === 'event') {
                    // One-shot event from Jetson (e.g. mission_completed, mission_faulted).
                    // Re-broadcast as mission_status without overwriting the telemetry cache.
                    console.log(`[jetson] event: ${parsed.event} id=${parsed.id || ''} fault=${parsed.fault || ''}`);
                    broadcast({ type: 'mission_status', event: parsed.event,
                                missionId: parsed.id, name: parsed.name, fault: parsed.fault });
                    // new_target events also get a dedicated broadcast so the web UI
                    // can refresh the target gallery without filtering mission_status
                    if (parsed.event === 'new_target') {
                        broadcast({ type: 'new_target', target_id: parsed.target_id });
                    }
                    if (parsed.event === 'learn_progress') {
                        broadcast({ type: 'learn_progress', target_id: parsed.target_id,
                                    count: parsed.count, total: parsed.total });
                    }
                } else if (parsed.type === 'scan') {
                    // LIDAR scan — forward directly, don't overwrite telemetry cache
                    broadcast({ type: 'scan', obs: parsed.obs, fwd: parsed.fwd, pts: parsed.pts });
                } else if (parsed.type === 'slam_map') {
                    // Occupancy grid PNG — cache so new connections get the current map
                    lastSlamMap = parsed;
                    broadcast(parsed);
                } else if (parsed.type === 'learn_crops') {
                    // Appearance crops from Jetson — call Claude for description
                    callClaudeDescription(parsed.target_id, parsed.crops);
                } else {
                    lastStatus = parsed;
                    broadcast({ type: 'status', data: lastStatus });
                }
            } catch (e) { console.warn('[jetson] JSON parse error:', e.message); }
        }
    });

    sock.on('close', () => {
        clearTimeout(connectTimer);
        if (jetsonOnline) {
            console.log('[jetson] Disconnected');
            broadcast({ type: 'connection', status: 'disconnected' });
            // Clear stale telemetry so reconnecting clients don't see stale data
            lastStatus  = null;
            lastSlamMap = null;
            broadcast({ type: 'status', data: null });
        }
        jetsonOnline  = false;
        jetsonSocket  = null;
        scheduleReconnect();
    });

    sock.on('error', (err) => {
        clearTimeout(connectTimer);
        console.error('[jetson] Socket error:', err.message);
        sock.destroy();   // triggers 'close'
    });
}

// ── AI appearance description ─────────────────────────────────────────────────
async function callClaudeDescription(target_id, crops) {
    if (!crops || !crops.length) {
        broadcast({ type: 'learn_status', target_id, status: 'error', message: 'No crops collected' });
        return;
    }
    if (!anthropic) {
        broadcast({ type: 'learn_status', target_id, status: 'error', message: 'API key not configured' });
        return;
    }
    console.log(`[learn] Sending ${crops.length} crops for person ${target_id} to Claude`);
    broadcast({ type: 'learn_status', target_id, status: 'analysing', count: crops.length });
    try {
        const content = [
            ...crops.map(b64 => ({
                type: 'image',
                source: { type: 'base64', media_type: 'image/jpeg', data: b64 }
            })),
            {
                type: 'text',
                text: 'Describe the appearance of the person in these images. Focus on: clothing (colours, styles, any logos or patterns), hair (colour, length, style), build, and any accessories such as bags, hats, glasses, or lanyards. Be specific and concise — 2-3 sentences.'
            }
        ];
        const response = await anthropic.messages.create({
            model: 'claude-haiku-4-5-20251001',
            max_tokens: 200,
            messages: [{ role: 'user', content }]
        });
        const description = response.content[0].text.trim();
        targetDescriptions.set(target_id, description);
        // Persist to disk so descriptions survive server restarts
        const obj = Object.fromEntries(targetDescriptions);
        fs.writeFile(DESCRIPTIONS_FILE, JSON.stringify(obj, null, 2), () => {});
        console.log(`[learn] Person ${target_id}: ${description}`);
        broadcast({ type: 'target_description', target_id, description });
    } catch (err) {
        console.error('[learn] Claude API error:', err.message);
        broadcast({ type: 'learn_status', target_id, status: 'error', message: err.message });
    }
}

// ── WebSocket ping/pong to drop stale browser connections quickly ──────────────
const WS_PING_INTERVAL = 20000;
const wsAlive = new WeakMap();

function heartbeat() { wsAlive.set(this, true); }

setInterval(() => {
    for (const ws of clients) {
        if (wsAlive.get(ws) === false) { ws.terminate(); continue; }
        wsAlive.set(ws, false);
        ws.ping();
    }
}, WS_PING_INTERVAL);

// ── WebSocket ─────────────────────────────────────────────────────────────────
wss.on('connection', (ws) => {
    clients.add(ws);
    wsAlive.set(ws, true);
    ws.on('pong', heartbeat.bind(ws));
    ws.send(JSON.stringify({ type: 'connection', status: jetsonOnline ? 'connected' : 'disconnected' }));
    if (lastStatus)  ws.send(JSON.stringify({ type: 'status', data: lastStatus }));
    if (lastSlamMap) ws.send(JSON.stringify(lastSlamMap));
    for (const [tid, desc] of targetDescriptions)
        ws.send(JSON.stringify({ type: 'target_description', target_id: tid, description: desc }));
    ws.on('message', (raw) => {
        try {
            const { cmd } = JSON.parse(raw);
            if (typeof cmd === 'string' && cmd.length > 0 && cmd.length <= 256) {
                const ok = sendToJetson(cmd);
                ws.send(JSON.stringify({ type: 'ack', cmd, ok }));
            }
        } catch (e) { console.warn('[ws] message parse error:', e.message); }
    });
    ws.on('close', () => { clients.delete(ws); wsAlive.delete(ws); });
    ws.on('error', () => { clients.delete(ws); wsAlive.delete(ws); });
});

// ── Mission API ───────────────────────────────────────────────────────────────
function missionFile(id) {
    return path.join(MISSIONS_DIR, id.replace(/[^a-zA-Z0-9_-]/g, '') + '.json');
}

app.get('/api/missions', (_req, res) => {
    try {
        const list = fs.readdirSync(MISSIONS_DIR)
            .filter(f => f.endsWith('.json'))
            .map(f => {
                try {
                    const d = JSON.parse(fs.readFileSync(path.join(MISSIONS_DIR, f)));
                    return { id: d.id, name: d.name, type: d.type, created: d.created, updated: d.updated, waypointCount: (d.waypoints || []).length };
                } catch { return null; }
            })
            .filter(Boolean)
            .sort((a, b) => (b.updated || '').localeCompare(a.updated || ''));
        res.json(list);
    } catch (e) { console.error('[api] missions list:', e.message); res.status(500).json({ error: 'Internal server error' }); }
});

// Return the last saved mission state from the Jetson's status (if available).
// MUST be registered before /:id so Express doesn't match 'state' as an id.
app.get('/api/missions/state', (_req, res) => {
    if (lastStatus && lastStatus.mission && lastStatus.mission.id) {
        res.json({
            mission_id:   lastStatus.mission.id,
            waypoint_idx: lastStatus.mission.waypoint_idx ?? 0
        });
    } else {
        res.json({});
    }
});

app.get('/api/missions/:id', (req, res) => {
    const fp = missionFile(req.params.id);
    if (!fs.existsSync(fp)) return res.status(404).json({ error: 'Not found' });
    res.json(JSON.parse(fs.readFileSync(fp)));
});

app.post('/api/missions', (req, res) => {
    try {
        const mission = req.body;
        if (!mission.id) mission.id = Date.now().toString(36) + Math.random().toString(36).slice(2, 6);
        if (!mission.created) mission.created = new Date().toISOString();
        mission.updated = new Date().toISOString();
        fs.writeFileSync(missionFile(mission.id), JSON.stringify(mission, null, 2));
        res.json({ ok: true, id: mission.id });
    } catch (e) { console.error('[api] mission save:', e.message); res.status(500).json({ error: 'Internal server error' }); }
});

app.put('/api/missions/:id', (req, res) => {
    try {
        const mission = { ...req.body, id: req.params.id, updated: new Date().toISOString() };
        fs.writeFileSync(missionFile(req.params.id), JSON.stringify(mission, null, 2));
        res.json({ ok: true });
    } catch (e) { console.error('[api] mission update:', e.message); res.status(500).json({ error: 'Internal server error' }); }
});

app.delete('/api/missions/:id', (req, res) => {
    const fp = missionFile(req.params.id);
    if (fs.existsSync(fp)) fs.unlinkSync(fp);
    res.json({ ok: true });
});

app.post('/api/missions/:id/push', (req, res) => {
    const fp = missionFile(req.params.id);
    if (!fs.existsSync(fp)) return res.status(404).json({ error: 'Not found' });
    const mission = JSON.parse(fs.readFileSync(fp));
    // Allow caller to supply route-expanded waypoints without overwriting the saved file
    if (req.body && Array.isArray(req.body.waypoints)) mission.waypoints = req.body.waypoints;
    const ok = sendToJetson(`mission_save:${JSON.stringify(mission)}`);
    broadcast({ type: 'mission_status', event: 'pushed', missionId: mission.id, name: mission.name });
    res.json({ ok });
});

app.post('/api/missions/:id/execute', (req, res) => {
    const fp = missionFile(req.params.id);
    if (!fs.existsSync(fp)) return res.status(404).json({ error: 'Not found' });
    const mission = JSON.parse(fs.readFileSync(fp));
    // Allow caller to supply route-expanded waypoints without overwriting the saved file
    if (req.body && Array.isArray(req.body.waypoints)) mission.waypoints = req.body.waypoints;
    const ok = sendToJetson(`mission_start:${JSON.stringify(mission)}`);
    broadcast({ type: 'mission_status', event: 'executing', missionId: mission.id, name: mission.name });
    res.json({ ok });
});

// ── Road routing via OSRM ─────────────────────────────────────────────────────
function fetchJson(url) {
    return new Promise((resolve, reject) => {
        const mod = url.startsWith('https') ? require('https') : require('http');
        const req = mod.get(url, { headers: { 'User-Agent': 'tank-c2/1.0' } }, res => {
            let d = '';
            res.on('data', c => d += c);
            res.on('end', () => { try { resolve(JSON.parse(d)); } catch(e) { reject(e); } });
        });
        req.on('error', reject);
        req.setTimeout(10000, () => req.destroy(new Error('OSRM timeout')));
    });
}

// Ramer-Douglas-Peucker simplification (degree units)
function rdpSimplify(pts, eps) {
    if (pts.length <= 2) return pts;
    let maxD = 0, maxI = 0;
    const [a, b] = [pts[0], pts[pts.length - 1]];
    const dx = b.lng - a.lng, dy = b.lat - a.lat;
    const len2 = dx*dx + dy*dy;
    for (let i = 1; i < pts.length - 1; i++) {
        const t = len2 ? Math.max(0, Math.min(1, ((pts[i].lng-a.lng)*dx + (pts[i].lat-a.lat)*dy) / len2)) : 0;
        const d = Math.hypot(pts[i].lng - a.lng - t*dx, pts[i].lat - a.lat - t*dy);
        if (d > maxD) { maxD = d; maxI = i; }
    }
    if (maxD > eps) {
        return [...rdpSimplify(pts.slice(0, maxI+1), eps).slice(0,-1),
                ...rdpSimplify(pts.slice(maxI), eps)];
    }
    return [a, b];
}

// Ensure no segment longer than maxDeg degrees (~45m at equator per 0.0004°)
function densify(pts, maxDeg) {
    const out = [pts[0]];
    for (let i = 1; i < pts.length; i++) {
        const a = pts[i-1], b = pts[i];
        const d = Math.hypot(b.lat-a.lat, b.lng-a.lng);
        if (d > maxDeg) {
            const n = Math.ceil(d / maxDeg);
            for (let j = 1; j < n; j++)
                out.push({ lat: a.lat+(b.lat-a.lat)*j/n, lng: a.lng+(b.lng-a.lng)*j/n });
        }
        out.push(b);
    }
    return out;
}

app.post('/api/route', async (req, res) => {
    const { points, profile = 'foot' } = req.body || {};
    if (!points || points.length < 2) return res.status(400).json({ error: 'Need ≥2 points' });
    // Map UI profile names to OSRM profiles
    const osrmProfile = profile === 'car' ? 'driving' : 'foot';
    const coords = points.map(p => `${p.lng},${p.lat}`).join(';');
    const url = `http://router.project-osrm.org/route/v1/${osrmProfile}/${coords}?overview=full&geometries=geojson`;
    try {
        const data = await fetchJson(url);
        if (data.code !== 'Ok' || !data.routes?.length)
            return res.status(400).json({ error: 'No route found' });
        const raw = data.routes[0].geometry.coordinates.map(([lng, lat]) => ({ lat, lng }));
        // Simplify (keep bends ≥3m) then densify (max 45m between points)
        const simplified = rdpSimplify(raw, 0.00003);
        const final = densify(simplified, 0.0004);
        res.json({ points: final, distance: data.routes[0].distance, duration: data.routes[0].duration });
    } catch (err) {
        console.error('[route]', err.message);
        res.status(502).json({ error: 'Routing service error' });
    }
});

app.post('/api/missions/abort', (_req, res) => {
    const ok = sendToJetson('mission_abort');
    if (ok) {
        broadcast({ type: 'mission_status', event: 'aborted' });
    } else {
        // Jetson is offline — queue the abort so it fires the moment it reconnects
        if (pendingCritical.length < PENDING_CRITICAL_MAX) {
            pendingCritical.push({
                cmd: 'mission_abort',
                broadcastOnDeliver: { type: 'mission_status', event: 'aborted' }
            });
        }
        broadcast({ type: 'mission_status', event: 'abort_pending' });
        console.warn('[jetson] Abort queued — will send on reconnect');
    }
    res.json({ ok, queued: !ok });
});

// ── Layout API (dashboard widget layouts) ─────────────────────────────────────
const LAYOUTS_DIR = path.join(__dirname, 'layouts');
if (!fs.existsSync(LAYOUTS_DIR)) fs.mkdirSync(LAYOUTS_DIR, { recursive: true });

function layoutFile(name) {
    return path.join(LAYOUTS_DIR, name.replace(/[^a-zA-Z0-9_-]/g, '') + '.json');
}

app.get('/api/layouts', (_req, res) => {
    try {
        const list = fs.readdirSync(LAYOUTS_DIR)
            .filter(f => f.endsWith('.json'))
            .map(f => {
                try {
                    const d = JSON.parse(fs.readFileSync(path.join(LAYOUTS_DIR, f)));
                    return { name: d.name, created: d.created, updated: d.updated, widgetCount: (d.widgets || []).length };
                } catch { return null; }
            })
            .filter(Boolean)
            .sort((a, b) => (b.updated || '').localeCompare(a.updated || ''));
        res.json(list);
    } catch (e) { console.error('[api] layouts list:', e.message); res.status(500).json({ error: 'Internal server error' }); }
});

app.get('/api/layouts/:name', (req, res) => {
    const fp = layoutFile(req.params.name);
    if (!fs.existsSync(fp)) return res.status(404).json({ error: 'Not found' });
    res.json(JSON.parse(fs.readFileSync(fp)));
});

app.put('/api/layouts/:name', (req, res) => {
    try {
        const layout = req.body;
        layout.name = req.params.name;
        if (!layout.created) layout.created = new Date().toISOString();
        layout.updated = new Date().toISOString();
        fs.writeFileSync(layoutFile(req.params.name), JSON.stringify(layout, null, 2));
        res.json({ ok: true });
    } catch (e) { console.error('[api] layout save:', e.message); res.status(500).json({ error: 'Internal server error' }); }
});

app.delete('/api/layouts/:name', (req, res) => {
    const fp = layoutFile(req.params.name);
    if (fs.existsSync(fp)) fs.unlinkSync(fp);
    res.json({ ok: true });
});

// ── WebSocket upgrade — authenticate via one-time token ───────────────────────
server.on('upgrade', (req, socket, head) => {
    const url = new URL(req.url, `http://localhost`);
    const tok = url.searchParams.get('tok');
    if (!tok || !validateWsToken(tok)) {
        socket.write('HTTP/1.1 401 Unauthorized\r\n\r\n');
        socket.destroy();
        return;
    }
    wss.handleUpgrade(req, socket, head, (ws) => wss.emit('connection', ws, req));
});

// ── Start ─────────────────────────────────────────────────────────────────────
server.listen(WEB_PORT, () => {
    console.log(`Tank C2 running on http://0.0.0.0:${WEB_PORT}`);
    connectJetson();
});
