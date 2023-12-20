# **Talon-Library**

**Talon-Library is a powerful FRC software library that provides developers with a comprehensive set of tools to speed up and simplify the robot software development process.** Whether you're a seasoned FRC developer or just starting out, Talon-Library's modular architecture and effective tools make it easy to write efficient and reliable code for your robot. With a focus on performance and ease of use, Talon-Library provides developers with a robust set of tools for interacting with FRC hardware without any hassle.
## Getting Started

To get started with Talon-Library, simply add it as a dependency to your project using the code snippets provided below. Once you've added Talon-Library to your project, take a look at the extensive documentation and examples to start using its powerful tools to develop your FRC robot software.

If you're new to FRC development or just getting started with Talon-Library, we recommend checking out the documentation for the Robot classes inside of [WPILib](https://docs.wpilib.org/en/stable/index.html) to learn how to set up your robot's hardware and define its behavior. Once you've got the basics down, you can start exploring Talon-Library's other features.

If you run into any issues or have any questions, don't hesitate to reach out. We're always happy to help and provide guidance to help you get the most out of Talon-Library.
### **Maven**

To include Talon-Library in your FRC project using Maven, add the following dependency to your pom.xml file:

```xml
  <dependency>
    <groupId>org.talonrobotics</groupId>
    <artifactId>talonlib</artifactId>
    <version>1.0.0-SNAPSHOT</version>
  </dependency>
```

### **Gradle**

To include Talon-Library in your FRC project using Gradle, add the following dependency to your build.gradle file:

#### **Groovy:**

```gradle
dependencies {
    implementation 'org.talonrobotics:talonlib:1.0.0-SNAPSHOT'
}
```

#### **Kotlin**

```gradle
dependencies {
    implementation("org.talonrobotics:talonlib:1.0.0-SNAPSHOT")
}
```

In addition to Maven and Gradle, it is reccomended that you have Java installed on your machine. Talon-Library is written in Java, so you will need to have a basic understanding of Java programming to use it. We recommend that you have some experience with FRC programming as well.
