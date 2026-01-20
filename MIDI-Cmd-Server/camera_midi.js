var easymidi = require('easymidi');
const axios = require('axios');

// Controller web interface URL (can be overridden via CONTROLLER_URL environment variable)
const CONTROLLER_URL = process.env.CONTROLLER_URL || 'http://10.10.10.163';

// Map MIDI notes to preset indices (controller supports 16 presets: 0-15)
// Preset 0 is the home position (0,0,0)
// Presets 1-15 are user-defined camera positions
const NOTE_TO_PRESET_MAP = {
    // Original camera positions (preserved from old mapping)
    127: 0,   // Home position
    1: 1,   // Preset 1
    2: 2,   // Preset 2
    3: 3,   // Preset 3
    4: 4,   // Preset 4
    5: 5,   // Preset 5
    6: 6,   // Preset 6
    7: 7,   // Preset 7
    8: 8,   // Preset 8
    9: 9,   // Preset 9
    10: 10,  // Preset 10
    11: 11, // Preset 11
    12: 12, // Preset 12
    13: 13, // Preset 13
    14: 14, // Preset 14
    15: 15, // Preset 15
};

async function recallPreset(presetIndex) {
    try {
        const response = await axios.post(`${CONTROLLER_URL}/api/preset/goto`, {
            index: presetIndex
        }, {
            headers: {
                'Content-Type': 'application/json'
            },
            timeout: 2000
        });
        
        if (response.data.status === 'ok') {
            console.log(`Successfully recalled preset ${presetIndex}`);
        } else {
            console.error(`Failed to recall preset ${presetIndex}:`, response.data.error || 'Unknown error');
        }
    } catch (error) {
        console.error(`Error recalling preset ${presetIndex}:`, error.message);
    }
}

async function init() {
    var input = new easymidi.Input('NewDay-Stream');
    input.on('noteon', async function (msg) {
        // Only process notes on channel 1 with velocity > 0
        if (msg.channel == 1 && msg.velocity > 0) {
            const presetIndex = NOTE_TO_PRESET_MAP[msg.note];
            
            if (presetIndex !== undefined && presetIndex !== null) {
                // Add a small delay before sending the command
                await sleep(400);
                await recallPreset(presetIndex);
            } else {
                console.log(`MIDI note ${msg.note} (velocity: ${msg.velocity}) - No preset mapping configured`);
            }
            
            if (presetIndex !== undefined) {
                console.log(`MIDI note ${msg.note} (velocity: ${msg.velocity}) -> Preset ${presetIndex}`);
            }
        }
    });

    console.log(`MIDI server initialized. Controller URL: ${CONTROLLER_URL}`);
    console.log('Waiting for MIDI input...');
};

function sleep(ms) {
    return new Promise((resolve) => {
        setTimeout(resolve, ms);
    });
}

init();
