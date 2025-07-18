from post_station.actions import action, _resolve_param, GRAVEYARD_SIGNAL

@action('conditional')
async def conditional_value(station, parcel, params):
    condition_param = params.get('condition')

    if condition_param is None:
        station.get_logger().error('conditional_value missing "condition" parameter')
        return GRAVEYARD_SIGNAL

    # Evaluate condition param (could be a static value or an action)
    condition = await _resolve_param(station, parcel, condition_param)
    if not isinstance(condition, bool):
        station.get_logger().error('conditional: condition did not resolve to bool')
        return GRAVEYARD_SIGNAL
    
    if condition:
        return await _resolve_param(station, parcel, params.get('if_true'))
    else:
        return await _resolve_param(station, parcel, params.get('if_false'))

