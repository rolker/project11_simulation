# Skip launch_testing files when run via plain pytest (colcon test).
# These require the launch_test runner instead.
collect_ignore = ['test_tide_integration.py']
