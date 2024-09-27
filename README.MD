# MTi Receive Latency

This code firstly use xspublic library to record a .csv file, then use python `plot_utc.py` to plot the data points w.r.t to timestamp to see if it is evenly distributed.


## Install Dependancy
```
pip3 install -r requirements.txt
```

## Other command

To compare the result, you could try the command:
```
setserial /dev/ttyUSB0 low_latency
```
or you could cancel it:
```
setserial /dev/ttyUSB0 ^low_latency
```
`setserial` reference [here](https://linux.die.net/man/8/setserial)

When you run the command `setserial /dev/ttyUSB0 low_latency` on Ubuntu 22.04, it configures the specified serial port (`/dev/ttyUSB0`) to operate in low latency mode. This command directly interacts with the kernel to adjust the settings of the serial port driver, specifically modifying the internal data structures that control the behavior of the serial port.

However, this command does not change any persistent configuration files on the filesystem. Instead, it modifies the in-memory settings of the serial port driver. The effect of this command will be lost after a reboot because it does not change any configuration files that are read during the boot process.


## Investigation

### Thread wait time due to conjureUpWaitTime in `datapoller.cpp`

```
int32_t DataPoller::conjureUpWaitTime(const XsByteArray& bytes) const
{
	if (bytes.size() == 0)
		return 3;
	else if (bytes.size() < 256)
		return 2;
	return 0;
}
```

### Set low latency in the code `serialinterface.cpp`

```
// Set low latency mode
struct serial_struct serinfo;
if (ioctl(m_handle, TIOCGSERIAL, &serinfo) < 0) {
    JLDEBUGG("TIOCGSERIAL failed");
} else {
    serinfo.flags |= ASYNC_LOW_LATENCY;
    if (ioctl(m_handle, TIOCSSERIAL, &serinfo) < 0) {
        JLDEBUGG("TIOCSSERIAL failed");
    }
}
```

