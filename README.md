cycloidal
=========

![ScreenShot](https://raw.github.com/mikedh/cycloidal/master/images/cycloidal_example.png)

Generate profiles for [cycloidal drives](https://en.wikipedia.org/wiki/Cycloidal_drive), a very cool reducer mechanism. They're popular in machine tools and robotics due to relative ease of fabrication, high ratios, decent efficiency, and ability to survive high shock loads. 

The profile generation was implemented from 'Gear geometry of cycloid drives', Chen BingKui et al.' and inspired by a [post on zincland](http://www.zincland.com/hypocycloid). 

Install requirements with:

```
pip install trimesh[easy] matplotlib
```

Run with:

```    
ipython -i cycloidal.py
```