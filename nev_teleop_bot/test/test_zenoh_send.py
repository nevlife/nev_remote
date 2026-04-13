#!/usr/bin/env python3
import argparse
import json
import time
import zenoh


def main():
    parser = argparse.ArgumentParser(description='Zenoh send test')
    parser.add_argument('--locator', default='tcp/127.0.0.1:7447')
    args = parser.parse_args()

    conf = zenoh.Config()
    conf.insert_json5('connect/endpoints', json.dumps([args.locator]))

    print(f'Connecting to {args.locator} ...')
    session = zenoh.open(conf)
    print('Connected!')

    pub = session.declare_publisher('nev/test/ping')
    seq = 0
    try:
        while True:
            payload = json.dumps({'seq': seq, 'ts': time.time()})
            pub.put(payload)
            print(f'[{seq}] sent: {payload}')
            seq += 1
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        pub.undeclare()
        session.close()
        print('Done')


if __name__ == '__main__':
    main()
