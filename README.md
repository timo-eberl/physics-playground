# Physics Playground

## Build and run

```
cmake -S . -B build/
cmake --build build/
./build/physics-playground
```

## Release build

```
cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release
cmake --build build_release/
./build_release/physics-playground
```
