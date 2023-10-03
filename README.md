# InventorsFtcLib

A repository containing various tools to easily use in your FTC teamcode.

The codebase heavily uses [FTCLib](https://github.com/FTCLib/FTCLib), [Road Runner](https://github.com/acmerobotics/road-runner) and [EasyOpenCV](https://github.com/OpenFTC/EasyOpenCV).

# A. Installation:
## Step 1:
Open up your FTC SDK Project in Android Studio. Locate and open the ```build.gradle(Module:Teamcode)``` file and add the following to the repositories section:
```Java
repositories {
    maven { url = 'https://jitpack.io' }
}
```

## Step 2:
Then add the following to the dependencies section:
```Java
dependencies {
    ...
    implementation 'com.github.theinventors-ftc:InventorsFtcLib:0.0.1-alpha'
}
```

## Step 3:
Run Gradle Sync in your SDK in order to sync with the 3rd party libraries.
