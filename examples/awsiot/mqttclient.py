import paho.mqtt.client as mqtt
import time
import psutil
import ssl

# go to the console and get your host  https://icebreaker.us-­east-­1.aws.amazon.com/icebreaker
host = "A2WZZ8RUQDA9FG.iot.us-east-1.amazonaws.com"

clientid = "myclientid"
topic ="$shadow/beta/state/MyThing"
payload = "None"
qos = "0"
retain = "False"
ca_certs ="certs/rootCA.pem"
certfile = "certs/cert.pem"
keyfile =  "certs/privateKey.pem"

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected to AWS with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(topic)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

# See if the message was sent correctly. Ideal for debugging
def on_publish(mosq, obj, mid):
    print("mid: " + str(mid))


# Set up the client
client = mqtt.Client(clientid)
client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish
client.tls_set(ca_certs, certfile, keyfile, cert_reqs=ssl.CERT_REQUIRED,tls_version=ssl.PROTOCOL_SSLv23, ciphers=None)
client.will_set(topic, payload=None, qos=0, retain=False)
# Connect
client.connect(host, 8883, 1)
run = True
while run:
    # Get the CPU load
    cpu_usage = psutil.cpu_percent(interval=None)
    print("CPU Usage %i" % cpu_usage)
    payload_content = "{\"state\":{\"reported\":{\"CPU\":\""+str(cpu_usage)+"\"}}}"
    # Send the CPU load
    client.publish(topic, payload_content)
    time.sleep(1)# sleep for 10 seconds before next call
print("Goodbye")

