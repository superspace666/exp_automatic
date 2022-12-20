# exp_automatic
### The course work code of Automatic Control Practice B
##### I designed a servo control system for the single axis screw slide table, to make it move to the right place that I need.
##### I used two method to control it: PID and frequency control method
<img src="https://user-images.githubusercontent.com/67407023/208691972-4ea9c415-c06a-41a6-9134-4ab6fe96b800.jpg" width="500" height="300">

### progress
##### First, I had a system identification on the device, with a chirp signal.
<img src="https://user-images.githubusercontent.com/67407023/208692962-40e45405-c01b-45b7-8dfd-d944fcfa962a.png" width="400" height="400">

##### Then, I used the system Identification tools in Matlab to get the system model.
<img src="https://user-images.githubusercontent.com/67407023/208693027-19ddce77-a63d-431d-a43b-144b4898bfb9.png" width="400" height="400">

##### Based on this system model, I was able to develop a algorithm to control the device:
<img src="https://user-images.githubusercontent.com/67407023/208693122-c4b9ed20-6246-4c5a-9163-519d47e569d4.png" width="700" height="300">

### Result of step response
##### We successfully make the 95% rise time within 0.1 second, with overshoot less than 10%, steady-state error less than 1mm
<img src="https://user-images.githubusercontent.com/67407023/208695078-10c01b08-62e7-4844-94b0-16cae6084f37.png" width="500" height="500">
