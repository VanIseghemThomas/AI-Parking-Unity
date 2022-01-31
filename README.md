# AI Parking in Unity
## A RL project focussed on autonomous parking, using Unity's ML-Agents toolkit.

This project was created to answer to following question:
**Is it possible to search for a parking spot and park in a simulation environment, using neural networks and park in complex scenario's?**

It's still in it's early stages, but autonomous driving is becoming a reality. It has lot's of advantages compared to humans. But one thing that is often getting overlooked is parking. A lot of people like myself are pretty good at driving in traffic, but fail when it comes to parking the car. It is and will stay a hard task for a lot of people. But what if this could be automated?

### So why the choice for a simulation environment? 
Because models from simulation are transferable to real life. Simulating gives us the advantage to speed up time, do things in parallel with lot's of cars, avoid expensive crashes and not having to create dangerous real life situations. Also it's a lot cheaper than buying and modifying a real car.

### Requirements

 - The project right here
 - Unity with the ML-Agents Toolkit installed
 - Python with PyTorch and ML-Agents

I highly suggest the usage of Anaconda.

## Installation
### 1) First clone the repository somewhere you like

### 2) Install Unity Hub
You can download Unity Hub from here: **[https://unity3d.com/get-unity/download](https://unity3d.com/get-unity/download)**
With Unity Hub you can easily manage Unity  installations and projects.

### 3) Install Unity
Load the Unity project and use the error message it gives regarding the version, to install the version of Unity the project was made in.
**![](https://lh5.googleusercontent.com/blqH-PccoH1khIPceOxbqgUS7LV2wxi8isphmjrslcVU76yY6FYkOOmTfYO4qwZoE9orvH5bxIdLIZ5INGW6p1DpA5BxnN5IBXfKdUwXZNUPJuRT8e3IlyMqCYaRMw)**
*The version in the picture is different from the actual project version*

### 4) Install the ML-Agents package in Unity
It could be that Unity already did this by looking at the manifest.json file. You can check this by going to *Window -> Package Manager*. Select *Packages: In Project* and ML-Agents 2.0.1 should be in there. If not, you can install this by selecting *Packages: Unity Registry* and searching for the ML-Agents package.

**![](https://lh3.googleusercontent.com/mSr30SKamTx5wViKCRiJB9w6lIcKu-JTIjOpUTYWQn_jMDHR_GeypubYaOQfNNhgHjBHQ7Bjf7LeI_MYtGfxfWRIaEcWIoP71blnO1ZoJZAvwdN_rGPoktHqJ6U9wg)**

### 5) Install Python with ML-Agents / PyTorch
I'll use Anaconda here.

    conda create -n <venv name> python=3.8
    conda activate <venv name>
    pip install mlagents

This can be different for your system depending on which platform you are, use the PyTorch website to create the install command for you. In my case (Windows/Nvidia) this was: 

    conda install pytorch torchvision torchaudio cudatoolkit=10.2 -c pytorch

Verify the installation by trying the command:

    mlagents-learn

This should give the Unity logo in the console:

**![](https://lh4.googleusercontent.com/Su_6IuXyVYrsI1Tk-t7-yVqJy83CxQUsUUt-Md6_JwfHzz898GUUb8wb0V3_E4354Qbn5ay2FcsVcgHqsg2aEsKrL_Q8VsL3Cz1gNV2mtCBQc2agwAmAf--Exf7-Hg)**
