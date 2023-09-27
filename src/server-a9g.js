const PORT = 4500;
const app = require('express')();
const http = require('http').Server(app);
const io = require('socket.io')(http);
const A9GSerialGPS = require('./a9g-serial-gps');

const _options = {
  device: process.argv[2],
  baudRate: parseInt(process.argv[3]),
};

const _a9gSerialGps = new A9GSerialGPS(_options);

_a9gSerialGps.on('error', console.error);

_a9gSerialGps.on('data', (data) => {
  let gpsState = _a9gSerialGps.getGpsState();
  io.emit('state', gpsState);
});

app.get('/', function (req, res) {
  res.sendFile(`${__dirname}/public/index.html`);
});

http.listen(PORT, function () {
  console.log(`listening on http://localhost:${PORT}`);
  _a9gSerialGps.start();
});