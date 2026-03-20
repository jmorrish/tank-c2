const express = require('express');
const { WebSocketServer } = require('ws');
const net = require('net');
const path = require('path');
const http = require('http');

// Jetson is reachable via Tailscale
const JETSON_HOST = process.env.JETSON_HOST || '100.91.47.30';
const JETSON_PORT = parseInt(process.env.JETSON_PORT || '9999');
const WEB_PORT    = parseInt(process.env.PORT || '3000');

const app    = express();
const server = http.createServer(app);
const wss    = new WebSocketServer({ server });

app.use(express.static(path.join(__dirname, 'public')));

// --- Jetson TCP bridge ---
let jetsonSocket  = null;
let jetsonBuffer  = '';
let lastStatus    = null;
let jetsonOnline  = false;
const clients     = new Set();

function broadcast(msg) {
    const str = JSON.stringify(msg);
    for (const ws of clients) {
        if (ws.readyState === 1) ws.send(str);
    }
}

function sendToJetson(cmd) {
    if (jetsonSocket && !jetsonSocket.destroyed && jetsonOnline) {
        jetsonSocket.write(cmd + '\n');
        return true;
    }
    return false;
}

function connectJetson() {
    console.log(`Connecting to Jetson at ${JETSON_HOST}:${JETSON_PORT}...`);
    jetsonSocket  = new net.Socket();
    jetsonBuffer  = '';
    jetsonOnline  = false;

    jetsonSocket.connect(JETSON_PORT, JETSON_HOST, () => {
        console.log('Connected to Jetson IPC');
        jetsonOnline = true;
        broadcast({ type: 'connection', status: 'connected' });
    });

    jetsonSocket.on('data', (data) => {
        jetsonBuffer += data.toString();
        let pos;
        while ((pos = jetsonBuffer.indexOf('\n')) !== -1) {
            const line = jetsonBuffer.slice(0, pos).trim();
            jetsonBuffer = jetsonBuffer.slice(pos + 1);
            if (!line) continue;
            try {
                lastStatus = JSON.parse(line);
                broadcast({ type: 'status', data: lastStatus });
            } catch (_) { /* ignore non-JSON lines */ }
        }
    });

    jetsonSocket.on('close', () => {
        console.log('Jetson disconnected — retrying in 3 s...');
        jetsonOnline = false;
        jetsonSocket = null;
        broadcast({ type: 'connection', status: 'disconnected' });
        setTimeout(connectJetson, 3000);
    });

    jetsonSocket.on('error', (err) => {
        console.error('Jetson socket error:', err.message);
        jetsonSocket.destroy();
    });
}

// --- WebSocket clients ---
wss.on('connection', (ws) => {
    clients.add(ws);

    // Send current state immediately
    ws.send(JSON.stringify({ type: 'connection', status: jetsonOnline ? 'connected' : 'disconnected' }));
    if (lastStatus) ws.send(JSON.stringify({ type: 'status', data: lastStatus }));

    ws.on('message', (raw) => {
        try {
            const { cmd } = JSON.parse(raw);
            if (cmd) {
                const ok = sendToJetson(cmd);
                ws.send(JSON.stringify({ type: 'ack', cmd, ok }));
                console.log(`CMD [${ok ? 'sent' : 'FAILED'}]: ${cmd}`);
            }
        } catch (_) {}
    });

    ws.on('close', () => clients.delete(ws));
});

server.listen(WEB_PORT, () => {
    console.log(`Tank C2 running on http://0.0.0.0:${WEB_PORT}`);
    connectJetson();
});
