# SLAM2REF: Advancing Long-Term Mapping with 3D LiDAR and Reference Map Integration for Precise 6-DoF Trajectory Estimation and Map Extension

#### [[Paper](https://link.springer.com/article/10.1007/s41693-024-001)] [[Data](https://mediatum.ub.tum.de/1743877)]


## What is SLAM2REF?
Pose-graph Multi-Session Anchoring with a Ground Truth map or with another LiDAR-based SLAM session.
- This project is an extension of [LT-SLAM](https://github.com/gisbi-kim/lt-mapper/tree/main/ltslam), which implements a custom GTSAM factor for anchoring (see BetweenFactorWithAnchoring.h). However, this project is completely ROS-independent.
- Moreover, we have implemented a novel Indoor Scan Context Descriptor for fast place recognition.
- Also, a novel YawGICP algorithm for robust point cloud registration with varying mostly yaw angles.
- SLAM2REF additionally allows the retrieval of 6-DoF poses with an accuracy of up to 3 cm given an accurate TLS point cloud as a reference map (this map should be accurate, at least regarding the position of permanent elements such as walls and columns).


    
## How to run the code?

For Building, add this flag to use only five threads `-j 5`; otherwise, the project might exit before building.

- After successfully building the project, all the input paths (to the query and central sessions, for example) and parameters are given in the file `config/params.yaml`.

- Then simply running the code (with the play bottom) should start the execution.
    in the console, the following should be visible:
  
    ```bash
    ----> Slam2ref starts.
    ```

- Once the program has been successfully finalized, you should see the following:
  
    ```bash
    ----> Slam2ref done.
    ```

## License
For academic usage, the code is released under the [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html). 

For any commercial purpose, please contact the author.

## Citation:
If you use this work or our data in your research, please include the following citations (these BibTeX entries are the best versions you will likely find ✔️).

**Paper & Data:**
The data consists of the BIM Model of [ConSLAM](https://github.com/mac137/ConSLAM) and Ground Truth poses.

```BibTeX
@article{SLAM2REF:vega2024:paper,
	title        = {{SLAM2REF}: advancing long-term mapping with {3D} {LiDAR} and reference map integration for precise 6-{DoF} trajectory estimation and map extension},
	author       = {Vega-Torres, Miguel A. and Braun, Alexander and Borrmann, André},
	year         = 2024,
	month        = 7,
	journal      = {Construction Robotics},
	publisher    = {Springer},
	volume       = 8,
	number       = 2,
	pages        = 13,
	doi          = {10.1007/s41693-024-00126-w},
	url          = {https://link.springer.com/article/10.1007/s41693-024-00126-w},
	notes        = {link to code: https://github.com/MigVega/SLAM2REF/. Link to data: https://mediatum.ub.tum.de/1743877},
	keywords     = {LiDAR; Multi-Session SLAM; Pose-Graph Optimization; Loop Closure; Long-term Mapping; Change Detection; {BIM} Update; {3D} Indoor Localization and Mapping},
	language     = {en}
}

@misc{SLAM2REF:vega2024:data,
	title        = {{ConSLAM} {BIM} and {GT} Poses},
	author       = {Vega-Torres, Miguel A. and Braun, Alexander and Borrmann, André},
	year         = 2024,
	month        = 6,
	publisher    = {Technical University of Munich},
	doi          = {10.14459/2024MP1743877},
	url          = {https://mediatum.ub.tum.de/1743877},
	type         = {Dataset},
	abstract     = {The ConSLAM BIM and GT Poses comprehends the 3D building information model (in IFC and Revit formats), manually elaborated based on the terrestrial laser scanner of the sequence 2 of ConSLAM, and the refined grounth truth (GT) poses (in TUM format) of the sessions 2, 3, 4 and 5 of the open-access Con{SLAM} dataset. This dataset can be found here: https://github.com/mac137/ConSLAM},
	keywords     = {LiDAR; Multi-Session SLAM; Pose-Graph Optimization; Loop Closure; Long-term Mapping; Change Detection; {BIM} Update; {3D} Indoor Localization and Mapping},
	language     = {en}
}
```
**Code:** To be added.

## Acknowledgements
This is an extension of [LT-SLAM](https://github.com/gisbi-kim/lt-mapper/tree/main/ltslam)(2022), whose author is Giseop Kim.
