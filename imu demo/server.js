/*
  Minimal static file server for IMU Biomechanics Lab
  Usage: node server.js
  Then open: http://localhost:3000
*/

const http = require('http');
const fs   = require('fs');
const path = require('path');

const PORT    = 3000;
const PUBLIC  = path.join(__dirname, 'public');

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.css':  'text/css',
  '.js':   'application/javascript',
  '.json': 'application/json',
  '.ico':  'image/x-icon',
};

http.createServer((req, res) => {
  let filePath = path.join(PUBLIC, req.url === '/' ? 'index.html' : req.url);
  // Security: prevent directory traversal
  if (!filePath.startsWith(PUBLIC)) {
    res.writeHead(403); res.end('Forbidden'); return;
  }
  const ext = path.extname(filePath);
  fs.readFile(filePath, (err, data) => {
    if (err) {
      if (err.code === 'ENOENT') { res.writeHead(404); res.end('Not found'); }
      else { res.writeHead(500); res.end('Server error'); }
      return;
    }
    res.writeHead(200, { 'Content-Type': MIME[ext] || 'application/octet-stream' });
    res.end(data);
  });
}).listen(PORT, () => {
  console.log(`\n  IMU Biomechanics Lab running at http://localhost:${PORT}\n`);
});
