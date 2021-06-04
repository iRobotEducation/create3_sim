# This message gives the status for a button

std_msgs/Header header                            # Header stamp is time and frame at which information was collected
bool is_pressed                                   # Whether or not button is depressed
builtin_interfaces/Time last_start_pressed_time   # The last time a button press was initiated
builtin_interfaces/Duration last_pressed_duration # The duration that the button was depressed
