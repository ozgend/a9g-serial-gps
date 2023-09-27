const A9GSerialGPS = require('./a9g-serial-gps');

const run = async () => {
  if (process.argv.length < 4) {
    console.log('Usage: node console-a9g.js <device> <baudrate>');
    const availablePorts = await A9GSerialGPS.listDevices();
    console.log('Available ports:');
    console.table(availablePorts);
    process.exit(0);
  }

  const options = {
    device: process.argv[2],
    baudRate: parseInt(process.argv[3]),
  };

  const a9gSerialGps = new A9GSerial(options);

  a9gSerialGps.on('error', console.error);

  a9gSerialGps.on('state', (state) => {
    console.log('State: ', state);
  });

  a9gSerialGps.on('data', (data) => {
    console.log('Data: ', data);
  });

  a9gSerialGps.start();
};

run();
