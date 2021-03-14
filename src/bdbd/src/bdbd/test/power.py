# reading power info
import time

VDD_CPU = '/sys/bus/i2c/drivers/ina3221x/6-0040/iio:device0/in_voltage2_input'
VDD_IN = '/sys/bus/i2c/drivers/ina3221x/6-0040/iio:device0/in_voltage0_input'
I_IN = '/sys/bus/i2c/drivers/ina3221x/6-0040/iio:device0/in_current0_input'

# https://www.linux-magazine.com/Issues/2018/217/Exploring-proc
def read_proc_cmdline(proc_filename):
    """
    Function to read the command-line of a process, given its proc_filename.
    Returns a tuple: first item of tuple is a boolean, True if successful,
    False if not; second item of tuple is the result (cmdline) if first item
    is True, or an error message if first item is False.
    """
    try:
        with open(proc_filename, 'r') as proc_fil:
            # Read cmdline value.
            data = proc_fil.read()
            # Make it printable.
            ret_val = (True, data.replace('\0', ' '))
    except IOError as ioe:
        ret_val = (False, "IOError while opening proc file: {} ".format(str(ioe)))
    except Exception as e:
        ret_val = (False, "Exception: {}".format(str(xe)))
    finally:
        return ret_val

voltsa = 0.0
ampsa = 0.0
factor = 0.1
count = 0
while True:
    try:
        v_in = read_proc_cmdline(VDD_IN)
        i_in = read_proc_cmdline(I_IN)
        volts = int(v_in[1]) / 1000.
        amps = int(i_in[1]) / 1000.
        voltsa = (1. - factor) * voltsa + factor * volts
        ampsa = (1. - factor) * ampsa + factor * amps
        watts = voltsa * ampsa
        if count % 10 == 0:
            print('VDD_IN {:6.2f} I_IN {:6.2f} Power {:6.2f}'.format(voltsa, ampsa, watts))
        count += 1
        time.sleep(.02)
    except KeyboardInterrupt:
        break
    except:
        print('io error')
