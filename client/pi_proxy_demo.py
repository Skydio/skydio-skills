"""
Example script to connect to R1 from a Raspberry Pi and request a video stream.

Use this in combination with gstreamer_proxy.py to send a stream to a different computer
via the Pi's WiFi.
"""
from __future__ import absolute_import
from __future__ import print_function
import argparse
import time

from skydio.comms.http_client import HTTPClient

# This url is the address of the vehicle from the usb-c port
USB_URL = 'http://192.168.11.1'

# The ip of a machine you want to proxy RTP packets to
REMOTE_HOST = '192.168.0.26'  # TODO: CHANGE ME TO MATCH YOUR SETUP!
REMOTE_PORT = 55004
# Run `python gstreamer_viewer.py --format jpeg` on that machine to view the video.

def main():

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseurl', default=USB_URL)
    parser.add_argument('--takeoff', action='store_true')
    parser.add_argument('--land', action='store_true')
    parser.add_argument('--repeat', action='store_true')
    parser.add_argument('--remote-host', type=str, default=REMOTE_HOST)
    args = parser.parse_args()


    # Ask for a 360x240 jpeg stream at 7.5fps
    stream_settings = {'source': 'NATIVE', 'port': REMOTE_PORT}

    # Acquire pilot access
    while True:
        try:
            client = HTTPClient(baseurl=args.baseurl,
                                pilot=True,
                                stream_settings=stream_settings)
            break
        except:
            print('failed to connect')
            if args.repeat:
                time.sleep(1)
            else:
                return

    # proxy RTP packets to a remote host
    subprocess.Popen(['python', 'gstreamer_proxy.py',
                      '--remote-host', args.remote_host,
                      '--remote-port', str(REMOTE_PORT)])

    if args.takeoff:
        print('taking off')
        client.takeoff()
    if args.land:
        print('landing')
        client.land()

    print('looping now')
    while True:
        print('status ping')
        client.update_pilot_status()
        time.sleep(2)
    print('done')


if __name__ == '__main__':
    main()
