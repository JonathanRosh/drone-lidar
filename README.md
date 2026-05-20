## Contributors

- Name: Ben Kogan  
  ID:  211868161
- Name: Jonathan Rosh 
  ID:  

## Build

Inside the course container from the project root run the following commands:

```text
cmake --presets default
cmake --build build

```

## Running the program

After building the project from the `build` directory you can run the following:

```text
./drone_mapper <input_output_files_path>
```

If you want to also add a `log.txt` file with simulation logs to the output you can add the `--log` flag:

```text
./drone_mapper <input_output_files_path> --log
```

## Running the tests

After building the project from the `build` directory you can run:

```text
ctest
```

## Input Format

The program reads three files from the input/output directory:

- `drone_config.txt`
- `mission_config.txt`
- `map_input.txt`

Configuration files use one `key: value` pair per line. Empty lines are allowed.
Lines whose first non-whitespace character is `#` are treated as comments. All
distances and lengths are in centimeters. Angles are in degrees.

### `drone_config.txt`

Required keys:

```text
min_pass_width: <number>
min_pass_height: <number>
min_pass_length: <number>
max_advance: <number>
max_elevate: <number>
max_rotate: <number>
lidar_z_min: <number>
lidar_z_max: <number>
lidar_d: <number>
lidar_fovc: <integer>
```

### `mission_config.txt`

Required keys:

```text
map_boundary_x_min: <number>
map_boundary_y_min: <number>
map_boundary_x_max: <number>
map_boundary_y_max: <number>
map_boundary_height_min: <number>
map_boundary_height_max: <number>
resolution_xy: <integer>
resolution_height: <integer>
```

The resolution keys describe the number of decimal places after the dot. For
example, `resolution_xy: 0` means one-centimeter XY grid spacing.

The parser also accepts these aliases:

```text
map_resolution_xy: <integer>
map_resolution_height: <integer>
```

Optional initial drone pose keys:

```text
initial_x: <number>
initial_y: <number>
initial_height: <number>
initial_xy_angle: <number>
```

If the initial pose is omitted, the drone starts at `(0, 0, 0)` with XY angle
`0`.

The parser also accepts these aliases:

```text
initial_position_x: <number>
initial_position_y: <number>
initial_position_height: <number>
initial_z: <number>
initial_position_z: <number>
initial_angle: <number>
```

### `map_input.txt`

The map input is a sparse list of occupied cells. Each non-comment line contains
one occupied coordinate:

```text
<x> <y> <height>
```

Commas may be used instead of spaces:

```text
<x>,<y>,<height>
```

Any in-bound coordinate that does not appear in this file is considered empty in
the true simulation map.

## Output Format

The program writes `map_output.txt` in the same sparse coordinate format as
`map_input.txt`. Each line contains one cell that the mapped output marks as
occupied:

```text
<x> <y> <height>
```

The program also prints the mapping score and related statistics to standard
output.
