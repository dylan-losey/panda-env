# Panda Environment

Simple PyBullet Environments for the Panda Robot Arm. Developed by the [ILIAD Lab](http://iliad.stanford.edu/), 
primarily by [Dylan Losey](https://dylanlosey.com/).

## Quickstart

Use these commands to quickly get set up with this repository, and start up the Panda Arm Environments.

```bash
# Clone the Repository
git clone https://github.com/dylan-losey/panda-env.git
cd panda-env

# Create a Conda Environment using `environment.yml`
conda env create -f environment.yml
```

## Start-Up (from Scratch)

Use these commands if the above quick-start instructions don't work for you, or you prefer to maintain your own Python
packaging solution, and want to make sure the dependencies are in check. If you're just trying to use the code, look at 
the Quickstart section above.

```bash
# Create & Activate Conda Environment
conda create --name panda-env python=3.7
conda activate panda-env

# Install Dependencies 
pip install pybullet numpy
```