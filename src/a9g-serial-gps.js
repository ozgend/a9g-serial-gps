const { SerialPort, ReadlineParser } = require('serialport');
const GPS = require('gps');
const Sylvester = require('sylvester');
const Kalman = require('kalman').KF;

const getAtCommand = (word) => {
  return `${word}\r\n`;
};


// Simple Kalman Filter set up
const _A = Sylvester.Matrix.I(2);
const _B = Sylvester.Matrix.Zero(2, 2);
const _H = Sylvester.Matrix.I(2);
const _C = Sylvester.Matrix.I(2);
const _Q = Sylvester.Matrix.I(2).multiply(1e-11);
const _R = Sylvester.Matrix.I(2).multiply(0.00001);

// Measure
const _uVector = $V([0, 0]);
const _kalmanFilter = new Kalman($V([0, 0]), $M([[1, 0], [0, 1]]));

class A9GSerialGPS {
  constructor({ device, baudRate = 115200, delimiter = '\r\n', pollInterval = 1 }) {
    this._device = device;
    this._baudRate = baudRate;
    this._delimiter = delimiter;
    this._pollInterval = pollInterval;
    this._gpsDataset = {};
    this._serialPort = null;
    this._eventHandlers = {};

    this._prev = { lat: null, lon: null };

    this._gps = new GPS();
    this._gps.state.bearing = 0;
    this._gps.on('data', data => {
      const type = data.type;
      this._gpsDataset[type] = data;

      if (this._prev.lat !== null && this._prev.lon !== null) {
        this._gps.state.bearing = GPS.Heading(this._prev.lat, this._prev.lon, this._gps.state.lat, this._gps.state.lon);
      }

      this._prev.lat = this._gps.state.lat;
      this._prev.lon = this._gps.state.lon;

      if (data.lat && data.lon) {
        _kalmanFilter.update({ A: _A, B: _B, C: _C, H: _H, R: _R, Q: _Q, u: _uVector, y: $V([data.lat, data.lon]) });

        this._gps.state.position = {
          cov: _kalmanFilter.P.elements,
          pos: _kalmanFilter.x.elements
        };
      }

      this._getEventHandler('data')(data);
    });

    setInterval(() => {
      this._getEventHandler('state')(this._gps.state);
    }, this._pollInterval * 1000);


    setInterval(() => {
      this._getEventHandler('dataset')(this._gpsDataset);
    }, this._pollInterval * 1000);
  }

  _getEventHandler(type) {
    let handler = this._eventHandlers[type] ?? (() => { });
    return handler;
  }

  static async listDevices() {
    return SerialPort.list();
  }

  on(type, handler) {
    this._eventHandlers[type] = handler
  }

  off(type, handler) {
    delete this._eventHandlers[type];
  }

  getGpsState() {
    return this._gps.state;
  }

  enableGps() {
    this._write('AT+GPS=1');
  }

  setGpsPolling(interval) {
    this._write(`AT+GPSRD=${interval}`);
  }

  disableGps() {
    this._write('AT+GPS=0');
  }

  _write(command) {
    if (!this._serialPort?.isOpen || !this._serialPort?.writable) {
      console.error('Serial port not open or not writable');
      this._getEventHandler('error')(new Error('Serial port not open or not writable'));
      return;
    }

    let atCommand = getAtCommand(command);
    this._serialPort.write(atCommand, (err) => {
      if (err) {
        console.error('Error on write: ', err.message);
        this._getEventHandler('error')(err);
      }
    });
  }

  start() {
    if (this._serialPort?.isOpen) {
      return;
    }

    this._serialPort = new SerialPort({
      path: this._device,
      baudRate: this._baudRate,
      autoOpen: false,
      parser: new ReadlineParser({
        delimiter: this._delimiter
      }),
    }, (err) => {
      if (err) {
        console.error('Error opening serial port: ', err.message);
        this._getEventHandler('error')(err);
      }
    });

    this._serialPort.on('data', data => {
      this._gps.updatePartial(data);
      this._getEventHandler('raw-serial')(data);
    });

    this._serialPort.open((err) => {
      if (err) {
        console.error('Error opening serial port: ', err.message);
        this._getEventHandler('error')(err);
        return
      }
      console.log('Serial port opened');
      this.enableGps();
      this.setGpsPolling(this._pollInterval);
    });
  }

  stop() {
    if (!this._serialPort || !this._serialPort.isOpen) {
      return;
    }

    this.setGpsPolling(0);
    this.disableGps();

    this._serialPort.close((err) => {
      if (err) {
        console.error('Error closing serial port: ', err.message);
        this._getEventHandler('error')(err);
      }
    });

    this._serialPort.destroy((err) => {
      if (err) {
        console.error('Error destroying serial port: ', err.message);
        this._getEventHandler('error')(err);
      }
    });
  }
};

module.exports = A9GSerialGPS;