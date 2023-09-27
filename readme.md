# a9g-serial-gps

a9g module serial gps processing with [`gps.js`](https://github.com/infusion/GPS.js) and `serialport`. map uses [`leaflet`](https://leafletjs.com/) with [OpenStreetMap](https://www.openstreetmap.org/), [Carto](https://carto.com/) and [Stadia](https://stadiamaps.com/).

- ai thinker [a9g board](https://docs.ai-thinker.com/en/gprs/a9g/boards)
- [ft232rl](https://ftdichip.com/products/ttl-232r-3v3/) usb to serial

```bash
$ npm i
$ npm run server -- /dev/tty.usbserial-XXXXX 115200
```
