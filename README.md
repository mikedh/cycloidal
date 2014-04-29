cycloidal
=========

Generate profiles for [cycloidal drives](https://en.wikipedia.org/wiki/Cycloidal_drive), a very cool reducer mechanism. They're popular in machine tools and robotics due to relative ease of fabrication, high ratios, decent efficiency, and ability to survive high shock loads. 

The profile generation was implemented from 'Gear geometry of cycloid drives', Chen BingKui et al.' and inspired by a [post on zincland](http://www.zincland.com/hypocycloid). 

Numpy is required, matplotlib is recommended. Try tuning the dimension and count parameters in generate_regular. generate_rolling generates the profiles for a more hypothetical but related mechanism. 

Run with:

    ```
    ipython -i cycloidal.py
    ```