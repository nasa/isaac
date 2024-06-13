\page analyst Analyst Notebook


# Starting the Analyst Notebook

**The jupyter notebooks will be able to access data that is in the `$HOME/data` and `$HOME/data/bags`, therefore, make sure all the relevant bag files are there**

**If you want to run the OCR, make sure there is a `$HOME/data/str` folder with all the data**

For the Analyst notebook to be functional, it needs to start side-by-side with the database and the IUI (ISAAC user interface).
To do so, the recommended method is to use the remote docker images, as:

	$ISAAC_SRC/scripts/docker/run.sh --analyst --mount --no-sim --remote
 
The ISAAC UI is hosted in: http://localhost:8080
The ArangoDB database is hosted in: http://localhost:8529
The Analyst Notebook is hosted in: http://localhost:8888/lab?token=isaac

# Intro Tutorials

Please follow all the tutorial to familiarize yourself with the available functions and to detect if something is not working properly.

## 1) Import Bagfile data to database (optional if using remote database)

Open the tutorial [here](http://localhost:8888/lab/tree/1_import_bagfiles.ipynb).

This tutorial covers how to upload bag files to a local database. Be aware that uploading large bag files might take a long time. If possible select only the time intervals/topic names that are required for analysis to speed up the process.


## 2) Read data from the Database

Open the tutorial [here](http://localhost:8888/lab/tree/2_read_database.ipynb).

This tutorial covers how to display data uploaded to the database. It contains some examples of the most common data type / topics.
You can filter the data that gets collected from the database using queries.

## 3) Export results to ISAAC user interface

Open the tutorial [here](http://localhost:8888/lab/tree/3_export_result_to_iui.ipynb).

This tutorial covers the available methods to visualize data in the ISAAC user interface (IUI).

Open the IUI 3D viewer [here](http://localhost:8080).


# Case study Tutorials

## Collecting simulation data to train a CNN + validate with ISS data

Open the tutorial \href[here](http://localhost:8888/lab/tree/build_CNN_with_pytorch.ipynb).

Here, we use simulation tools to automatically build a train and test dataset. The simulation dataset builder uses arguments as target position model positions and gaussian noise to build.
Using the simulated data, we use pytorch to train the classifier of a previously trained CNN. We optimize the CNN using the train dataset, and use the test dataset to decide which iteration of the optimization to keep.
With the trained CNN we can run new colledted data through it, namely real image captured data.


## Building a volumetric map of WiFi signal intensity + analyse model + visualize data

Open the tutorial

