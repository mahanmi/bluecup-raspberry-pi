import { mkdirSync, existsSync, writeSync, write, writeFileSync } from 'fs';
import {join, resolve} from 'path';

function generateFileStructure(fileStruct) {
  let lastIndex = 0, lastFile = '', currentDir = './'
  for (const line of fileStruct.split('\n')) {
      const exe = line.match(/[^^\W*]\S+/)
      if (!exe) continue
      const {index, 0: fileName} = exe
  
      if (index > lastIndex) {
        currentDir = join(currentDir, lastFile)
        console.log(currentDir, lastFile)
      }
      if (index < lastIndex) {
        currentDir = join(currentDir, '../')
        console.log(currentDir, '../')
      }
      if (!existsSync(resolve(currentDir, fileName))) {
        if (fileName.endsWith('/')) 
          mkdirSync(resolve(currentDir, fileName))
        else
          writeFileSync(resolve(currentDir, fileName), '')
      }
      lastIndex = index
      lastFile = fileName
  }
}

generateFileStructure(`rov_controller/
├── gui/                     # GUI elements and logic
│   ├── main_window.py
│   ├── widgets/
│   └── assets/
├── api/                     # API endpoints and handling
│   ├── endpoints.py
│   └── schemas.py           # Data validation schemas (e.g., Pydantic)
├── robot_core/              # Core robot logic
│   ├── robot.py             # Main robot class, state management
│   ├── control.py           # Movement, manipulator control functions
│   ├── telemetry.py         # Sensor data handling
│   └── command_processor.py # Handles commands from GUI/API
├── hardware_interface/      # Communication with actual hardware
│   ├── motors.py
│   ├── sensors.py
│   ├── camera.py
│   └── communication.py     # Serial, UDP, etc.
├── common/                  # Shared utilities, constants
│   ├── utils.py
│   └── config.py            # Configuration settings
├── tests/                   # Unit and integration tests
│   ├── test_api.py
│   ├── test_robot_core.py
│   └── ...
├── main.py                  # Main application entry point
└── README.md
└── requirements.txt`)