# Ingest Customized map to CARLA

This is the map package that contains all necessary files for RoadRunner customize map ingestion. This guide is intended for CARLA source build only. Before import, make sure all three files (i.e., .fbx, .xodr, .rrdata.xml) share  the same map name.

**Step one** 

Copy the entire "map_package" file to the following directory 
``` directory
../carla/import
```

**Step two** 

Navigate to the CARLA root directory ```../carla``` and run the following command 
``` terminal
make import
``` 

A json file ```map_package.json``` should be generated automatically at ```../carla/import``` 

**Step three** 

Launch CARLA UE4 by running 
```terminal 
make launch
``` 

Locate the ```File``` tab at main toolbar in UE4 editor and select ```File → Open Level``` to import the customize map.


![image](https://user-images.githubusercontent.com/74442573/113051729-4ed8d880-9174-11eb-9753-dcfb87fd90f6.png)

Navigate the to the directory: ```Content/map_package/maps/<your_map_name>.umap```, and double click the ```.umap``` file to import the map. 


![image](https://user-images.githubusercontent.com/74442573/113051751-55675000-9174-11eb-90b8-fa1ed57897c6.png)


Once the map is loaded, click ```play``` to launch simulation. 

![image](https://user-images.githubusercontent.com/74442573/113051797-61531200-9174-11eb-865b-9660f82d1434.png)
