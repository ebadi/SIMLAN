Dyno-robotics/Infotiv delivery

Data collection from cameras according to [this requirements](20240522-req.png)

To reproduce

```
git checkout ....
# In three separate terminals:
./start.sh clean ; ./start.sh build ; ./start.sh sim
./start.sh camera_dump
./start.sh move_object
```
