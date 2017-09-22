# Hand_writing-in-the-air-with-Leap-Motion

Fingertip tracking/ Hand writing in the air/ Project the 3D space coordinates into 3D space plane points, then to 2D space plane.

## 1.How to import the Leap Motion Module with Python???
It can be solved by the methods described in the official website of Leap Motion: 
https://developer.leapmotion.com/documentation/v2/python/devguide/Project_Setup.html#id12;

Here, I keep the libraries in a seperate directory from my source code, and then add the path to Leap Motion libraries to the Python sys.path list before importing the Leap module.

## 2.The processes of trajectory projection of hand writing digits/letters from 3D to 2D.
2.1 Tracking the fingertip of index finger with Leap Motion to obtain the trajectory of hand writing digits/letters.<br>
2.2 Projecting 3D trajectory on the 3D space plane.<br>
1). The 3D space plane is made by three points obtained from the trajectory, which are the start point, the middle point and the end point of the trajectory, respectively.<br>
2). Computing the coordinates of projection points of the trajectory on the 3D space plane, and then the results of projection can be got.<br>

2.3 Transfering the projection results on 3D space plane into 2D plane.<br>
1). Finding two unit vectors that are perpendicular to each other on the 3D space plane by using above three points.<br>
2). Changing the 3d coordinates into 2d coordinates on the plane made by the two unit vectors.<br>

Here are some results of the three steps described above:

        e.g.1:The projection of letter 'a'
![3D trajectory](https://github.com/zttara/Hand_writing-in-the-air-with-Leap-Motion/blob/master/Examples%20of%20projection%20results/letter-a/a-original2.png)-->
![3D space plane](https://github.com/zttara/Hand_writing-in-the-air-with-Leap-Motion/blob/master/Examples%20of%20projection%20results/letter-a/a-3D%20plane1.png)-->
![2D plane](https://github.com/zttara/Hand_writing-in-the-air-with-Leap-Motion/blob/master/Examples%20of%20projection%20results/letter-a/a-2D%20plane.png)

        e.g.2:The projection of digit '2'
![3D trajectory](https://github.com/zttara/Hand_writing-in-the-air-with-Leap-Motion/blob/master/Examples%20of%20projection%20results/number-2/n2-1.png)-->
![3D space plane](https://github.com/zttara/Hand_writing-in-the-air-with-Leap-Motion/blob/master/Examples%20of%20projection%20results/number-2/n2-2.png)-->
![2D plane](https://github.com/zttara/Hand_writing-in-the-air-with-Leap-Motion/blob/master/Examples%20of%20projection%20results/number-2/n2_3.png)

## 3. Further studies
1). To find a more proper space plane for projection;<br>
2). To correct the direction of projection results;<br>
3). The further recognition of projection results.<br>
