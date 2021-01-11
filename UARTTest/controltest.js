const SerialPort = require('serialport');

async function run(){
 


    const port = new SerialPort('COM3', {
        baudRate: 115200
    });
    
    port.open(()=>{
        port.on('data', function (data) {
            console.log('Data:', data[0])
         })
         var i=150;
         port.write([i,i,i,i]);
        
    
    });
    
}

run();