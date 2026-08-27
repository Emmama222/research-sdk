# research-sdk

This is a tool set that would provide the basics to perform further development and evaluation. 

Current Project available : 
- Path Planning System Comparison (2026) 


## What does `research-sdk` provides ? 

This software provides : 

- offline sandbox simulation (point to point 2D Simulation)

< Pictures will be added soon >

- Connection to grSim Simulation 

< Pictures will be added soon >

- Connection to TurtleRabbit - Phenoix server
< Pictures will be added soon >

(Future adaptation)


- World model and map produced from What was received over the network from Small Size League Vision System. 

- basic robot control function wrapper. 

## Installation and dependencies
This software is designed as a local python software. Please run the following to install. Using a virtual environment is highly recommended.

### Installing and activate Virtual Environment
```shell
python3 -m venv .venv 
source .venv/bin/activate # Linux or MacOS
source .venv/Scripts/activate # Windows
```

```shell
pip install -e .[dev]
```

