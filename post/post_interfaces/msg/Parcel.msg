# post_interfaces/msg/Parcel.msg

string owner_id            # Creator/owner of the parcel
string parcel_id           # Unique ID for this parcel

string current_location    # Name of the station currently holding the parcel
string next_destination    # Target station to send the parcel next

string current_state       # Logical state of the parcel
string instruction_set_id  # Identifier for the instruction set to use

uint32 ttl                # Time-to-live hops remaining
bool error                # Whether an error occurred during processing
string error_reason       # Optional error description
