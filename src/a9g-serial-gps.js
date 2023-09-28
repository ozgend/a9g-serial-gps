const { SerialPort, ReadlineParser } = require('serialport');
const GPS = require('gps');
const Sylvester = require('sylvester');
const e = require('express');
const Kalman = require('kalman').KF;

const ignoredAtCommands = ['GPSRD'];

const getAtCommand = (word) => {
  return `${word}\r\n`;
};


/****/
// position filtering and smoothing
// via https://github.com/infusion/GPS.js

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
/****/

class A9GSerialGPS {
  static async listDevices() {
    return SerialPort.list();
  }

  constructor({ device, baudRate = 115200, delimiter = '\r\n', pollInterval = 2 }) {
    this._device = device;
    this._baudRate = baudRate;
    this._delimiter = delimiter;
    this._pollInterval = pollInterval;
    this._gpsDataset = {};
    this._serialPort = null;
    this._eventHandlers = {};
    this._atResponses = {};

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

      this._getEventHandler('gps.data')(data);
    });

    // report gps.state periodically
    setInterval(() => {
      this._getEventHandler('gps.state')(this._gps.state);
    }, this._pollInterval * 1000);

    // report gps.dataset periodically, for debugging
    setInterval(() => {
      this._getEventHandler('gps.dataset')(this._gpsDataset);
    }, this._pollInterval * 1000);

    // always report at.* responses
    setInterval(() => {
      this._getEventHandler(`at.*`)(this._atResponses);

      Object.entries(this._atResponses).forEach(([atKey, atResponse]) => {
        this._getEventHandler(`at.${atKey}`)(atResponse);
      })
    }, 1000);

    // poll signal quality
    setInterval(() => {
      this._write('AT+CSQ');
    }, 3000);

    // poll device info
    setInterval(() => {
      this._write('AT+GPSMD?');
    }, 10000);
  }

  _getEventHandler(type) {
    let handler = this._eventHandlers[type] ?? (() => { });
    return handler;
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

  disableGps() {
    this._write('AT+GPS=0');
  }

  setGpsPolling(interval) {
    this._write(`AT+GPSRD=${interval}`);
  }

  setAtCommandResponse(command, response) {
    if (ignoredAtCommands.some(c => command.includes(c))) {
      return;
    }
    this._atResponses[command] = response;
  }

  _write(command) {
    if (!this._serialPort?.isOpen || !this._serialPort?.writable) {
      console.error('Serial port not open or not writable');
      this._getEventHandler('error')(new Error('Serial port not open or not writable'));
      return;
    }

    let atCommand = getAtCommand(command);
    this._serialPort.write(atCommand, (err) => {
      this.setAtCommandResponse(command, 'NO_RESPONSE');
      if (err) {
        console.error('Error on write: ', err.message);
        this._getEventHandler('error')(err);
        this.setAtCommandResponse(command, err.message);
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
      // handle AT responses for non AT+GPSRD data
      const stringData = data.toString().trim();
      const firstLine = stringData.split('\r\n')[0];
      const sentCommands = Object.keys(this._atResponses);
      if (!firstLine.includes('+GPSRD') && sentCommands.includes(firstLine)) {
        //console.log(stringData);
        const atCommand = firstLine;
        const atResponseLines = stringData.split('\r\n');
        let atResponse = 'NO_RESPONSE';

        atResponse = atResponseLines.find(line => line.startsWith('+'));

        if (atResponse) {
          atResponse = atResponse.replace(/\r\n/g, '').split(':').pop().trim();
        }
        else {
          atResponse = atResponseLines.find(line => line.startsWith('OK') || line.startsWith('ERROR'))
        }

        this.setAtCommandResponse(atCommand, atResponse);
      }
      // handle AT+GPSRD data
      else {
        this._gps.updatePartial(data);
      }

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