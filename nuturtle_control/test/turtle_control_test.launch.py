<launch>

    <!-- Specific launch arguments can also be included at the user's discretion -->
    <arg 
        name='test_duration'
        default='2.0'
        description='Max length of test in seconds.'
    />

    <!-- Auxiliary nodes can be run like normal to test integration between nodes -->
    <node pkg='nuturtle_control' exec='turtle_control'>
        <param from="$(find-pkg-share nuturtle_description)/config/diff_params.yaml"/>
    </node>

    <!--
    catch2_integration_test_node
    a wrapper around node which passes the "result_file" argument to Catch2.
    There should only be one integration test node. This node will shutdown
    the entire launch file when it exits.
    Specific parameters and other arguments can also be passed, like the
    "test_duration" example below.
    -->
    <catch2_integration_test_node
        pkg='nuturtle_control'
        exec='turtle_control_test'
    >
        <param name='test_duration' value='$(var test_duration)'/>
    </catch2_integration_test_node>
</launch>