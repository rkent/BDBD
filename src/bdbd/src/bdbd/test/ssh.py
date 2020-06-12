import paramiko
import logging
logging.basicConfig(level=logging.DEBUG)

client = paramiko.SSHClient()
client.load_system_host_keys()
client.connect('ubutower.dryrain.org')
