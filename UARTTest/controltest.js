const SerialPort = require('serialport');

async function run(){
    const res = await SerialPort.list();
    console.log(res.map((e)=>e.path + ' - ' + e.manufacturer).join("\n"));


    const port = new SerialPort('COM3', {
        baudRate: 115200
    });
    
    port.open(()=>{
        port.on('data', function (data) {
            console.log('Data:', data[0])
         })
         var i=0;
         setInterval(() => {
            port.write([i,i,i,i]);
            if(i>= 255){
                i = 0;
            }
            i++;
         }, 10);
    
    });
    
}

run();