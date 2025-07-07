string owner_id         # Owner of the baton
string baton_id         # Unique ID per owner

BatonRouteStop[] route  # Ordered list of stops

uint32 ttl             # Time-to-live hops remaining
bool loop              # Loop route if true

uint32 route_index     # Current position in the route
