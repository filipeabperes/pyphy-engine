Improved version of pyphy-engine, used in "Learning Visual Predictive Models of Physics for Playing Billiards, ICLR 2016" ([arXiv](http://arxiv.org/abs/1511.07404)) to simulate the billiards environments. 


### Usage
1. output with default parameters (log for position and velocity)
```
python main.py --outDir result/
```
2. output frames
```
python main.py --outDir result/ --outIm True
```
3. change dynamics: friction and collision damp
```
python main.py --outDir result/ --outIm True --aFric 10 --aDamp 0.5
```
4. change setup: ball (num, size, mass, init state), table (shape)
```
python main.py --outDir result/ --outIm True --ballNum 2 --ballRadius 10 --ballMass 15 --ballInit 5 --tableType 0 
```
Additional parameters `--fps` and `--numFrames` can be used to control the frames per second in the simulation and the number of frames the simulation is run for.

### Improvement
- fixed bugs for multiple balls
- add friction parameter
- add collision damping parameter
