
# RoadRunner

**Contents**


* [Preparing GPS data](#preparing-gps-data)

* [Config File](#config-file)

* [Run RoadRunner](#Run-RoadRunner)

* [Post-processing with Map-matching(optional)](#Post-processing-with-Map-matching)

* [Merge RoadRunner with Other Solutions](#Merge-RoadRunner-with-Other-Solutions)

* [Example](#Example)



### Preparing GPS Data
Related source code:

* the GPSTraceServer folder (The code is in GO)

In RoadRunner, we need to handle several queries on GPS traces. The GPS dataset can be very large, ranging from 10 GB to 100+ GB. To speed up these queries, we create index for the raw GPS data.

You have to create two folders, one for the raw GPS trips, the other is used to store GPS index. E.g.,

```bash
mkdir PATH_TO_FOLDERS/trip_folder/
mkdir PATH_TO_FOLDERS/index_folder/
``` 

In the trip folder, each GPS trip should be stored in one file. The format of the file name is "trip_ID.txt". E.g., trip_1.txt 

In each trip file, each line is one GPS sample. The format is,
```
local_id,lat,lon,timestamp
```

(**NO EXTRA SPACE**)

The local_id here can be just the line number. 

E.g.,

```
1566,42.220984,-71.180424,1487518570
1567,42.22103,-71.180381,1487518571
1568,42.221048,-71.180399,1487518572
1569,42.221095,-71.180414,1487518573
1570,42.221054,-71.180481,1487518574
```

Next, we create index for these GPS trips. In GPSTraceServer folder, we use create_index.go to create the index. 

```
go run create_index.go trip_folder output_index_folder
```


After the index is created, you can start the query server.

```
go run trace_server.go index_folder trip_folder max_trips
```

For the index_folder and trip_folder, you have to add '/' at the end of the folder name. The max_trips can be just a very large number such as 10000000.

The Roadrunner code will interact with this trace server through TCP. 

You have to install the following package for the trace server. 
```
go get "github.com/ajstarks/svgo"
```

### Config File

To run RoadRunner, you need to prepare a config file. The config file is in json format. It contains information such as the path to the GPS data, the bounding box and the starting locations. Below, we show an example of this config file. (example_config.json) 

```json
{
  "Entrances": [
    {
      "Lat": 34.06805726555864, 
      "Lon": -118.26319758992187, 
      "ID": 1
    }, 
    {
      "Lat": 34.06803059744141, 
      "Lon": -118.26522533991407, 
      "ID": 2
    }, 
    {
      "Lat": 34.06801281869658, 
      "Lon": -118.25882022485938, 
      "ID": 3
    }, 
    {
      "Lat": 34.06801281869658, 
      "Lon": -118.24246947889063, 
      "ID": 4
    }, 
    {
      "Lat": 34.06800392932417, 
      "Lon": -118.25377767196875, 
      "ID": 5
    }, 
    {
      "Lat": 34.067977261206934, 
      "Lon": -118.26378767589844, 
      "ID": 6
    }, 
    {
      "Lat": 34.0679505930897, 
      "Lon": -118.26018278702344, 
      "ID": 7
    }
  ], 
  "CNNInput": null, 
  "Exits": [], 
  "Region": [
    34.031761958, 
    -118.279998947, 
    34.0681728274, 
    -118.236053635
  ], 
  "RegionMask": null, 
  "OutputFolder": "PATH_TO_OUTPUT_FOLDER",
  "history_length": 4,
  "minimal_number_of_trips_deferred_branch": 5,
  "minimal_number_of_trips": 2
}



``` 
In this config file, you have to provide the starting locations where RoadRunner algorithm will start from. We usually put these starting locations on the boundaries of the region. 

You may have to tune some of the parameters to make different tradeoffs between precision and recall. 



### Run RoadRunner

You have to first start the GPS trace server before running the following command. 

```
python RoadRunner.py config.json output_prefix
```

This program will dump the output (the pickle of the **RoadForest** class) to the output folder every 1000 iterations. It will stop until all valid edges have been added to the graph. 

You can convert the **RoadForest** format graph to a vertices/edges format using RoadForest2RoadGraph.py. E.g.,

```
python RoadForest2RoadGraph.py roadrunner_pickle_output graph_output
```

RoadForest2RoadGraph.py takes the **RoadForest** class pickle file as input and generates a new pickle file with vertices/edges format. 

You can refer to line 316 in RoadGraph.py for the details of this format. You may need to use this format for visualization and evaluation. Basically, self.nodes and self.edges should contain most of the information you need. 


### Post-processing with Map-matching

TODO

### Merge RoadRunner with Other Solutions

TODO

### Example

A template folder structrue is shown in the 'example' folder. There are 5 fake GPS trajectories and a config file. You could use this folder as a starting point. 