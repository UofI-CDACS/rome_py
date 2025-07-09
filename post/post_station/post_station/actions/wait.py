import asyncio
from post_station.actions import action

@action('wait')
async def wait_action(node, parcel, params):
    seconds = params.get('seconds', 1)
    await asyncio.sleep(seconds)

