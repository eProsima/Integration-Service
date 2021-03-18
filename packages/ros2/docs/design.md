# Design

## Ros2 type inference: compilation and load phases.
![](type-inference.png)

## Sequence diagram
![](http://www.plantuml.com/plantuml/png/jLR1Rjim3BthAuXUsXs20Zi6yz3JiEE0xYx8CXCJKIHFajjbNpyhKuXOaBV6M7Eo8Zw-9pxIrbE8oiVnQ8HGEZeFBlRePUZpaeBKpgCOfYZID2WRuVaK8Xw_AjkP-SQNC-Oaj3C6TIHd0yJJWD9G6mJQIF44NXiz348q3P2bIChGBvLYeMb4feFDvndB5hIpEzgF7fCkUb4HvpAk5WcTMROWPIAHyk73OeXr7KxRd3SBBDRmUF-zfm0U1uy1RGmGUmIBhw0zJd7Tj9uUkll7ETEdpOQfGbHn3AB3agpKbkim8L1vtJ_SUHS-th58P5zAAjv874MiSMw0dVEWZ06FFqRos9qD4Z3z5ZNio7RoY26e_TdAgLWfliZFIQfw-EmfgAAj9Y2tqpONM2Qb9NUEINzUdXEKksuBDHHlpt19RW5akIMmVbXMNC6oLBGTzta2pHkTMwJUvyiJeheNz972MoUahEumjeP2ZpxDGgcvNjgZEjHmiphnq_N8BLFoH33kRGEquE6t2uEechXiO1MOTz_3lJ2sGNjgrxh7-5ljO-0A_nZxVWO-CyMj3ZBoFrlOD10TfFTn-c3zKJyhB9ivH79MQcLCg_JX7-KjKcXdqpZBwEJF_0Qx3kNFz1Oeo3bVUyN4SOKT3AySbo7vR7D7651p8nLdvzsHotF9LPmk3_wgahXKavH_GbURiqY-lYLP_8qLUEt98FZHMhBxlzvgma1MEgkJlRVTQzoXShxRxnthnq0yeUtIlV4t)

## Future work
*eProsima* is planning to replace this whole system with a brand new one, that includes a C++ parser for **msg** and **srv** types.
This parser will automatically generate a dynamic representation of the type using [eprosima xTypes](https://github.com/eProsima/xtypes),
so no conversion library will be needed for each specific type and we can remove the code generation part and the dynamic library load
process of *.mix* files for each ROS 2 type that it is desired to use in the intercommunication process between the *ROS 2 SystemHandle* and other plugins.


