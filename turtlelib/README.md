# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- `geometry2d`: handles 2D geometry primitives
- `se2d`: handles 2D rigid body transformations
- `svg`: draws points, vectors, and frames in a .svg file
- `frame_main` (executable): performs some rigid body computations based on user input

# Conceptual Questions
##### 1. If you needed to be able to normalize Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - **Propose three different designs for implementing the normalize functionality**
      * <ins>Design 1</ins>: A function could be added to the Vector2D struct which simply divides the current fields x and y by their magnitude. 
  
      * <ins>Design 2</ins>: A helper function could be written within the turtlelib namespace that takes in a Vector2D object by value, copies it, and returns a new normalized one.
  
      * <ins>Design 3</ins>: An operator overloading function for division "/" could be incorporated into the struct which would divide the vector fields by some scalar value. This could then easily be used for vector normalization through another explicit normalization function inside the class.
    
  - **Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.** 
    * <ins>Design 1</ins>: This approach modifies the existing Vector2D object in place, making it spatially efficient. It also keeps the normalization logic relevant to the struct wrapped inside one unit, which embraces the idea of encapsulation. It does open Vector2D objects to be modified, however. The C++ core guidelines encourage using immutable data over mutable data to avoid any unexpected behavior (P.10)
  
    * <ins>Design 2</ins>: This approach preserves the original object in the struct, keeping it immutable. A Vector2D object only contains two floats. This is copied cheaply and is likely faster than having any redirection involved with passing an object in by reference (F.16). Moreover, it is also encouraged to place helper functions in namespaces where their purpose is useful to the class/struct (C.5). 
  
    * <ins>Design 3</ins>: Operator overloading is convenient when it is intuitive to the user. If not, it can easily be misused. This, in combination with this approach making class data being mutable, is undesirable.

   - **Which of the methods would you implement and why?**
    I would opt for the second design. It is the simplest solution, which the C++ core guidelines is often in favor of. Normalizing a vector can also be useful within the turtlelib namespace across testing and implementing other functionalities.

##### 2. What is the difference between a class and a struct in C++?
The difference lies in the default accessibility levels; in structs it is public while in classes it is private. Structs are typically used for simple data structures with less functionality while classes involve more encapsulation and complex behavior.

##### 3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
Vector2D is a simple data structure with related data. It also has no invariant conditions or variables, so it would be unnecessary for it to be a class (C.1, C.2). 

Transform2D on the other hand has private fields which should not be handled lightly. Modifying them by accident could be detrimental to the program and create unexpected behavior. Having it be a class enables the rotation and translation fields to be read only through their getter methods. (C.8, C.9).


##### 4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

Single argument constructors are expected to be explicit (C.46) in order to prevent unintended conversions. If the constructor is not explicit, things like function parameters could accidentally be converted to class objects if if the type of the argument matches the constructor's single parameter.


##### 5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?

A const function cannot modify the internal state of an object unless it has mutable fields. Transform2D::inv() does not modify the current object, but rather, a copy of it. Transform2D::operator*=() *does* modify the object's current fields, so it cannot be const.