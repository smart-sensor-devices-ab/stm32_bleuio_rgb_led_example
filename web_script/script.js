import * as my_dongle from 'bleuio'
const dongleToConnect='[0]40:48:FD:E5:2F:17'
document.getElementById('connect').addEventListener('click', function(){
  my_dongle.at_connect()
  document.getElementById("connect").disabled=true;
  document.getElementById("sendMsgForm").hidden=false;
})

document.getElementById("sendMsgForm").addEventListener("submit", function(event){
    event.preventDefault()

    
    my_dongle.ati().then((data)=>{
        //make central if not
        if(JSON.stringify(data).includes("Peripheral")){
            console.log('peripheral')
            my_dongle.at_central().then((x)=>{
                console.log('central now')
            })
        }        
    })
    .then(()=>{
        // connect to dongle
        my_dongle.at_getconn().then((y)=>{
            if(JSON.stringify(y).includes(dongleToConnect)){
                console.log('already connected')
            }else{
                my_dongle.at_gapconnect(dongleToConnect).then(()=>{
                    console.log('connected successfully')
                })
            }
        })
        .then(()=>{
            var theVal = document.getElementById('msgToSend').value;
            console.log('Message Send  '+theVal)
            // send command to show data
            my_dongle.at_spssend(theVal).then(()=>{
                console.log('Message Send '+theVal)
            })
        })
        
    })
  });



