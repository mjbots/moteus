# Python API reference

## moteus.Controller

::: moteus.Controller
    options:
        members:
            - set_position
            - make_position
            - set_position_wait_complete
            - set_stop
            - make_stop
            - query
            - make_query
            - custom_query
            - make_custom_query
            - set_output_nearest
            - make_set_output_nearest
            - set_output_exact
            - make_set_output_exact
            - set_recapture_position_velocity
            - make_recapture_position_velocity
            - set_vfoc
            - make_vfoc
            - set_current
            - make_current
            - set_stay_within
            - make_stay_within
            - set_brake
            - make_brake
            - set_write_gpio
            - make_write_gpio
            - read_gpio
            - set_trim
            - make_set_trim
            - set_aux_pwm
            - make_aux_pwm

## moteus.make_transport_args

::: moteus.make_transport_args

## moteus.get_singleton_transport

::: moteus.get_singleton_transport

## moteus.QueryResolution

::: moteus.QueryResolution
    options:
        show_if_no_docstring: true

## moteus.PositionResolution

::: moteus.PositionResolution
    options:
        show_if_no_docstring: true

## moteus.Stream

::: moteus.Stream
    options:
        members:
            - command
            - write_message
            - read_data

## moteus.Transport

::: moteus.Transport
    options:
        members:
            - cycle
            - read
            - discover
            - flush_read_queue
            - devices
            - count
            - close

## moteus.FdcanusbDevice

::: moteus.FdcanusbDevice
    options:
        members:
            - detect_fdcanusbs

## moteus.PythonCanDevice

::: moteus.PythonCanDevice
    options:
        members:
            - enumerate_devices
