# Alex to the Rescue!
CG1112 Engineering Principles and Practice II Final Project by Group B02-2A.

## Background (Adopted from Course Material)
72 hours. That is the "golden period" to locate and rescue survivors in the aftermath of
natural / manmade disasters such as earthquake, landslide and terrorist attack. Against the
ticking clock, rescuers have to brave incredible difficulties like rubbles / debris, narrow /
impassable passages and / or hazardous environment to look for any sign of life. Fortunately,
recent robotic advancement opens up many new possibilities for the rescue team.

## Group Members
| *Name* | *Role* |
| ------------- | ------------- |
| [Prof Boyd Anderson](https://github.com/boydanderson) | Instructor |
| [Darren Loh Rui Jie](https://github.com/saintmist21) | To be decided |
| [Hoang Trong Tan](https://github.com/jushg) | To be decided |
| [Hu Jialun](https://github.com/SuibianP) | To be decided |
| [Zhu Zikun](https://github.com/zikunz) | To be decided |

## Objectives, Algorithms and Design Features
### Main Functionality – Environment Mapping
To be added

### Additional Functionality
To be added

## Development
### `make` Targets
#### general
- `format`: format code with `clang-format`
- `lint`: lint code with `clang-tidy` (some analysis unavailable since clang support for avr is experimental)
- `clean`: clean files generated during compilation
#### `src`
- `ide`: Copy and manipulate arduino sources for compilation with Arduino IDE. **Usage strongly discouraged** since it defeats all good intentions of switching to GNU Make.
#### `arduino`
- `flash` (default): Flash onto board
#### `pi`
- `client` (default): Compile client program

### Prerequisites
- A basically POSIX-compliant system (basically anything except for Microsoft Windows®, excluding virtual machine and WSL)
- `make`: build utility
- `avr-gcc`: gcc compiler for avr
- `avr-objcopy`: object file utility
- `avrdude`: flash onto board
- `clang-tidy`: lint code
- `clang-format`: format code
