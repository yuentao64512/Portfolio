import asyncio, sys
from amqtt.broker import Broker
async def main():
    b = Broker({"listeners": {"default": {"type": "tcp", "bind": "127.0.0.1:18830"}},
                "sys_interval": 0, "auth": {"allow-anonymous": True, "plugins": ["auth_anonymous"]},
                "topic-check": {"enabled": False}})
    await b.start(); print("broker up", flush=True)
    await asyncio.Event().wait()
asyncio.run(main())
