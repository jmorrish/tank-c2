const express = require('express');
const { WebSocketServer } = require('ws');
const net = require('net');
const path = require('path');
const http = require('http');
const fs = require('fs');

const JETSON_HOST = process.env.JETSON_HOST || '100.91.47.30';
const JETSON_PORT = parseInt(process.env.JETSON_PORT || '9999');
const WEB_PORT    = parseInt(process.env.PORT || '3000');
const MISSIONS_DIR = path.join(__dirname, 'missions');

if (!fs.existsSync(MISSIONS_DIR)) fs.mkdirSync(MISSIONS_DIR, { recursive: true });

const app    = express();
const server = http.createServer(app);
const wss    = new WebSocketServer({ server });

app.use(express.json({ limit: '1mb' }));
app.use(express.static(path.join(__dirname, 'public')));

// ── Jetson TCP bridge ─────────────────────────────────────────────────────────
let jetsonSocket    = null;
let jetsonBuffer    = '';
let lastStatus      = null;
let jetsonOnline    = false;
let jetsonReconnecting = false;          // guard against parallel reconnect attempts
const CONNECT_TIMEOUT_MS = 8000;
const RECONNECT_DELAY_MS = 3000;
const clients = new Set();

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
        // TCP keepalive: detect silent drops (e.g. Tailscale blip) within ~15 s
        sock.setKeepAlive(true, 5000);
        broadcast({ type: 'connection', status: 'connected' });
    });

    sock.on('data', (data) => {
        jetsonBuffer += data.toString();
        let pos;
        while ((pos = jetsonBuffer.indexOf('\n')) !== -1) {
            const line = jetsonBuffer.slice(0, pos).trim();
            jetsonBuffer = jetsonBuffer.slice(pos + 1);
            if (!line) continue;
            try { lastStatus = JSON.parse(line); broadcast({ type: 'status', data: lastStatus }); } catch {}
        }
    });

    sock.on('close', () => {
        clearTimeout(connectTimer);
        if (jetsonOnline) {
            console.log('[jetson] Disconnected');
            broadcast({ type: 'connection', status: 'disconnected' });
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
    if (lastStatus) ws.send(JSON.stringify({ type: 'status', data: lastStatus }));
    ws.on('message', (raw) => {
        try {
            const { cmd } = JSON.parse(raw);
            if (cmd) {
                const ok = sendToJetson(cmd);
                ws.send(JSON.stringify({ type: 'ack', cmd, ok }));
                console.log(`[cmd] ${ok ? 'sent' : 'QUEUED'}: ${cmd.slice(0, 80)}`);
            }
        } catch {}
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
    } catch (e) { res.status(500).json({ error: e.message }); }
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
    } catch (e) { res.status(500).json({ error: e.message }); }
});

app.put('/api/missions/:id', (req, res) => {
    try {
        const mission = { ...req.body, id: req.params.id, updated: new Date().toISOString() };
        fs.writeFileSync(missionFile(req.params.id), JSON.stringify(mission, null, 2));
        res.json({ ok: true });
    } catch (e) { res.status(500).json({ error: e.message }); }
});

app.delete('/api/missions/:id', (req, res) => {
    const fp = missionFile(req.params.id);
    if (fs.existsSync(fp)) fs.unlinkSync(fp);
    res.json({ ok: true });
});

app.post('/api/missions/:id/execute', (req, res) => {
    const fp = missionFile(req.params.id);
    if (!fs.existsSync(fp)) return res.status(404).json({ error: 'Not found' });
    const mission = JSON.parse(fs.readFileSync(fp));
    const ok = sendToJetson(`mission_start:${JSON.stringify(mission)}`);
    broadcast({ type: 'mission_status', event: 'executing', missionId: mission.id, name: mission.name });
    res.json({ ok });
});

app.post('/api/missions/abort', (_req, res) => {
    const ok = sendToJetson('mission_abort');
    broadcast({ type: 'mission_status', event: 'aborted' });
    res.json({ ok });
});

// ── Start ─────────────────────────────────────────────────────────────────────
server.listen(WEB_PORT, () => {
    console.log(`Tank C2 running on http://0.0.0.0:${WEB_PORT}`);
    connectJetson();
});
