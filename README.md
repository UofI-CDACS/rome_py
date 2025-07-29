# POST - Parcel Oriented Station Transport

A ROS2-based system for routing and processing parcels through a network of stations using configurable instruction sets.

## Overview

POST is a distributed message processing system where "parcels" (data packets) are routed between "stations" (processing nodes) according to programmed "instruction sets". Each parcel can carry data such as routing information, and each specifies which instruction set to execute at each station.

### Key Concepts

- **Parcel**: A message containing data, a unique id, a previous and next location, an owner, and an instruction set to follow.
- **Station**: A ROS2 node that processes parcels according to their instruction sets
- **Instruction Set**: Defines the processing logic for parcels at stations
- **Actions**: Reusable processing units that can be composed into instruction sets

## Installation

### Prerequisites
- ROS2 (Jazzy)
- colcon build tools

### Build Instructions

1. Clone the repository into your ROS2 workspace:
```bash
cd ~/test_ws/src
git clone <repository_url>
```


2. Build the packages:
```bash
colcon build --packages-select post_interfaces post_core
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Quick Start

### Launch Multiple Stations

Launch 4 interconnected (by the loop instruction set) stations in the `faux` namespace:

```bash
ros2 launch post_core multi_station_launch.py namespace:=faux
```

This creates stations: `/faux/rospi_1`, `/faux/rospi_2`, `/faux/rospi_3`, `/faux/rospi_4`

### Launch an Individual Station

Launch 1 station on a machine with the `faux` namespace:

```bash
ros2 run post_core station --name default_station --type default --ros-args -r __ns:=/faux
```

### Send Test Parcels

Create and send parcels using the sender type station:
```bash
ros2 run post_core station --type sender --name test_sender
```
By default this will send 20 parcels to rospi_1 - rospi_4 in a round robin fashion, with the loop instruction set.

Configure the sender with parameters:
```bash
ros2 run post_core station --type sender --name station_sender --ros-args \
  #The destination nodes to send to.
  -p destinations:="['rospi_1','rospi_2','rospi_3','rospi_4']" \
  #The mode for the sender to use.
  #  round_robin will loop through the destinations until count is reached.
  #  random will pick a random one out of destinations until count is reached
  #  once will go through each of the destinations once until it reaches the end or until count is reached
  -p mode:=round_robin
  #The number of messages to send before stopping
  -p count:=20
  #The interval in seconds at which to send messages
  -p interval_sec:=0.1
  #The instruction set each parcel will use:
  -p instruction_set:=loop
#A ROS argument that sets the namespace for this station and therby the namespace it will send messages on unless the destinations are fully qualified names (/faux/rospi_1)
  -r __ns:=/faux
```

Watch the logs to see parcels moving through stations:
```bash
ros2 topic echo /faux/rospi_1/parcels
```

## Station Types

### Default Station (`default`)
- Processes parcels according to their specified instruction set
- Sends parcels to graveyard on error or specific returns from instruction sets.

### Sender Station (`sender`)
- Generates and sends parcels to specified destinations
- Configurable via ROS parameters
- Supports round-robin, random, or one-shot sending modes

### Graveyard Station (`graveyard`)
- Final destination for expired or error parcels
- Logs received parcels for analysis

## Instruction Sets

### Default (`default`)
- Decrements TTL counter
- Logs parcel to file
- Forwards to same station name (self-loop)
- Sends to graveyard when TTL expires

### Loop (`loop`)
- Routes parcels in a fixed cycle: rospi_1 → rospi_2 → rospi_3 → rospi_4 → rospi_1
- Decrements TTL and logs activity
- Sends to graveyard when TTL expires

### Loop Dynamic (`loop_dynamic`)
- Routes parcels according to a configurable route stored in parcel data
- Route defined as JSON array: `["rospi_1", "rospi_3", "rospi_2"]`
- Tracks current position with `route_index` in parcel data

### Graveyard (`graveyard`)
- Simple logging instruction set for terminal parcels
- No forwarding, just records the parcel
- Graveyard station automatically uses this instead of any instruction set set on the parcels it recieves

## Actions

Actions are reusable processing units:

- **`decrement_data_key`**: Decreases an integer value in parcel data
- **`check_ttl`**: Validates TTL is greater than zero
- **`forward`**: Sets next destination for parcel routing
- **`file_log_parcel`**: Writes parcel details to log file
- **`wait`**: Introduces processing delays

## Configuration

### Parcel Data Structure

```yaml
owner_id: "sender_station"
parcel_id: "uuid-string"
prev_location: "/faux/rospi_1" 
next_location: "/faux/rospi_2"
instruction_set: "loop"
data:
  - key: "ttl"
    value: "10"
```

### Station Parameters

#### Sender Station Parameters
- `destinations`: Array of target station names
- `count`: Number of parcels to send
- `mode`: Sending pattern (`round_robin`, `random`, `once`)
- `interval_sec`: Delay between sends
- `owner_id`: Parcel owner identifier
- `instruction_set`: Default instruction set for parcels
- `data`: Default key-value data for parcels

### Log Files

Parcels are logged to files organized by instruction set and station:
```
~/test_ws/station_type
└── namespace/as/path
    └── station_name
        └── log-parcel_owner-machine_hostname.txt
```

Log format:
```
timestamp,station_name,parcel_id,owner_id,prev_location,next_location,instruction_set,data_kvp
```

## Examples

### Simple Loop Network

1. Launch stations:
```bash
ros2 launch post_core multi_station_launch.py
```

2. Send a looping parcel:
```bash
ros2 run post_core station --type sender --name loop_sender --ros-args \
  -p destinations:="['rospi_1']" \
  -p mode:=once \
  -p instruction_set:=loop \
  -p data:="['ttl:5']" \
  -p count:=1 \
  -r __ns:=/faux
```

The parcel will visit: rospi_1 → rospi_2 → rospi_3 → rospi_4 → rospi_1 → ... until TTL expires.

### Dynamic Routing

1. Launch stations and create a custom route parcel:
```bash
ros2 launch post_core multi_station_launch.py
```

```bash
ros2 run post_core station --type sender --name loop_sender --ros-args \
  -p destinations:="['rospi_1']" \
  -p mode:=once \
  -p instruction_set:=loop_dynamic \
  -p data:="['ttl:8','route:[\"rospi_1\",\"rospi_3\",\"rospi_2\",\"rospi_4\"]','route_index:0']" \
  -p count:=1 \
  -r __ns:=/faux
```

The parcel follows: rospi_1 → rospi_3 → rospi_2 → rospi_4 → rospi_1 → ...

## Extending the System

### Creating New Instruction Sets

1. Create a new instruction set class:
```python
from post_core.post_instruction_sets import InstructionSet, register_instruction_set
from post_core.post_instruction_sets.types import InstructionResult, InstructionSignal

@register_instruction_set("my_custom_set")
class MyCustomInstructionSet(InstructionSet):
    async def run(self, station, parcel) -> InstructionResult:
        # Your processing logic here
        return InstructionResult(signal=InstructionSignal.CONTINUE)
```

2. Import in `post_core/post_instruction_sets/instruction_sets/__init__.py`

### Creating New Actions

1. Create action function:
```python
from post_core.post_actions.registry import register_action

@register_action('my_action')
async def my_action(station, parcel, param1, param2=default):
    # Your action logic here
    return result
```

2. Import in `post_core/post_actions/actions/__init__.py`

### Creating New Station Types

1. Inherit from Station base class:
```python
from post_core.post_stations import Station, register_station

@register_station("my_station_type")
class MyStation(Station):
    async def parcel_callback(self, parcel):
        # Custom parcel processing
        pass
```

## Troubleshooting

### Common Issues

1. **Parcels not routing**: Check namespace consistency in station names and destinations
2. **TTL expiring too quickly**: Increase TTL value in parcel data
3. **Instruction set not found**: Ensure instruction set is registered and imported
4. **Log files not created**: Check permissions for log directory (`~/test_ws/`)

### Debug Commands

```bash
# List active nodes
ros2 node list

# Monitor parcel topics
ros2 topic list | grep parcels
ros2 topic echo /namespace/station_name/parcels

# Check station parameters
ros2 param list /station_name
ros2 param get /station_name parameter_name
```
