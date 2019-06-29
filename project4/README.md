The aim of this project is to simulate the FMCW radar data with a "target" - a moving car, for instance. This data is then processed using FFT to get Range Doppler Map (RDM). In order to filter out the noise I’ve applied 2D CA-CFAR - 2D Cell Averaging Constant False Alarm Rate.
CA-CFAR consists of the following steps:
- First we define a 2D sliding window consisting of three types of cells - training cells, guard cells and cell under test (CUT). We will use data from training cells in order to predict the noise level for the CUT. The purpose of guard cells is to prevent data leakage from CUT into training cells;
- Next we calculate the average noise level for all the training cells to obtain the threshold;
- After that we calculate the signal level in the CUT and compare it to the threshold from the previous step. If it is larger then we assign to the respective cell in the 2D matrix the value of 1, otherwise - 0.

Since not all cells will be examined and any non examined cell is considered to have value equal to 0 I’ve decided to create an empty matrix of the same size as RDM and populate it with zeros before applying sliding window to the RDM.


CA-CFAR parameters seems to be not very robust. Current parameters work fine for the initial position and velocity set up in the script. However reducing the offset by 1, for instance, produces a lot of false positives. The same happens after changing the target’s initial parameters.
