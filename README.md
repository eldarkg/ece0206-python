### Description
Elcus ECE-0206-1С (ARINC429-USB) device user-space driver.
About device: [link to description](http://www.elcus.ru/pribors.php?ID=ece-0206-1c).

### Install
```sh
sudo python3 setup.py install
```

### Test
Attach device with stub to device connector.
Run:
```sh
python3 -m ece0206
```

### Dependency
* [python3 >= 3.5](https://www.python.org)
* [libusb1 >= 1.6.2](https://pypi.python.org/pypi/libusb1)
* [libusb-1.0 >= 1.0.21](http://libusb.info)
