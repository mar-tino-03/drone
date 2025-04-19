    // Connessione WebSocket
    const ws = new WebSocket('ws://' + window.location.hostname + '/ws');

    ws.onopen = () => {
        //console.log('WebSocket connected');
        document.getElementById('connected').innerText = "Connessione";
    };

    ws.onclose = () => {
        //console.log('WebSocket disconnected');
        document.getElementById('connected').innerText = "Disconnessione";
    };

/*invio["motor_1"] = map(motor_1, 0, 255, 0, 100);
    invio["motor_2"] = map(motor_2, 0, 255, 0, 100);
    invio["motor_3"] = map(motor_3, 0, 255, 0, 100);
    invio["motor_4"] = map(motor_4, 0, 255, 0, 100);
    invio["input_THROTTLE"] = input_THROTTLE;
    invio["roll_desired_angle"] = roll_desired_angle;
    invio["pitch_desired_angle"] = pitch_desired_angle;
    invio["yaw_desired_angle"] = yaw_desired_angle;
    invio["angle_roll_output"] = angle_roll_output;
    invio["angle_pitch_output"] = angle_pitch_output;
    invio["angle_yaw_output"] = angle_yaw_output;*/

/*TODO: dns/ipstatico, multi wifi(libreria),togliere banda da pagina.
 output: batteria, connessione. bottoni: volostatico*/

    ws.onmessage = (event) => { // TODO:
        var json = JSON.parse(event.data);
        /*for (var key in json){
            document.getElementById(key).innerText = json[key];
        }*/
        document.getElementById("batteria").innerText = json["batteria"].toFixed(2);
    } 

    var altitudineBloccata = false;

    //joystick
    class Joystick {
        constructor(container, joystickId) {
            this.container = container;
            this.joystick = container.querySelector('.joystick');
            this.containerRect = this.container.getBoundingClientRect();
            this.center = {
                x: this.containerRect.width / 2,
                y: this.containerRect.height / 2,
            };
            this.activeTouchId = null;
            this.joystickId = joystickId;
            this.timeCounter = 0;

            window.addEventListener('resize', () => {
                this.containerRect = this.container.getBoundingClientRect();
                this.center = {
                    x: this.containerRect.width / 2,
                    y: this.containerRect.height / 2,
                };
            });

            this.container.addEventListener('touchstart', this.start.bind(this), false);
            this.container.addEventListener('touchmove', this.move.bind(this), false);
            this.container.addEventListener('touchend', this.end.bind(this), false);
            this.container.addEventListener('touchcancel', this.end.bind(this), false);

            if(joystickId == "j1"){
                this.joystick.style.top = '85%';
            }
        }

        start(event) {
            for (let touch of event.changedTouches) {
                if (this.activeTouchId === null) {
                    this.activeTouchId = touch.identifier;
                    event.preventDefault();
                    return;
                }
            }
        }

        move(event) {
            for (let touch of event.changedTouches) {
                if (touch.identifier === this.activeTouchId && this.timeCounter>100) {
                    event.preventDefault();
                    
                    const rect = this.container.getBoundingClientRect();
                    const touchX = touch.clientX - rect.left;
                    const touchY = touch.clientY - rect.top;

                    const dx = touchX - this.center.x;
                    const dy = touchY - this.center.y;

                    const maxDistance = (this.container.offsetWidth - this.joystick.offsetWidth) / 2;
                    const distance = Math.min(Math.sqrt(dx * dx + dy * dy), maxDistance);

                    const angle = Math.atan2(dy, dx);
                    const joystickX = this.center.x + Math.cos(angle) * distance;
                    const joystickY = this.center.y + Math.sin(angle) * distance;

                    this.joystick.style.left = `${joystickX}px`;
                    this.joystick.style.top = `${joystickY}px`;

                    
                    // Invia i dati del joystick via WebSocket
                    var joystickData = {
                        [`${this.joystickId}X`]: 0,
                        [`${this.joystickId}Y`]: 0,
                    }; //(dx / maxDistance).toFixed(2)
                    if(this.joystickId == "j1"){
                        joystickData[`${this.joystickId}X`] = ((joystickX-75.0)/5).toFixed(2);
                        joystickData[`${this.joystickId}Y`] = (255+(-2.55*(joystickY-25.0))).toFixed(2);
                            //(dx / maxDistance).toFixed(2)
                    }else{
                        joystickData[`${this.joystickId}X`] = ((joystickX-75.0)/5).toFixed(2);
                        joystickData[`${this.joystickId}Y`] = ((-1*(joystickY-75.0))/5).toFixed(2);
                         //(dx / maxDistance).toFixed(2)
                    }
                    ws.send(JSON.stringify(joystickData));
                    /* joistickX = 0 - 100 (non quadrati)*/

                    //document.getElementById('input1').value = joystickData[`${this.joystickId}X`];

                    this.timeCounter = 0;

                    return;
                }else{
                    this.timeCounter += new Date().getMilliseconds();
                }
                /*document.getElementById('log').value += touch.identifier+ " ! "; 
                document.getElementById('log').value += this.activeTouchId;*/
            }
        }

        end(event) {
            for (let touch of event.changedTouches) {
                if (touch.identifier === this.activeTouchId) {
                    this.activeTouchId = null;
                    this.joystick.style.left = '50%';
                    this.joystick.style.top = '50%';   
                    
                    var joystickData = {
                        [`${this.joystickId}X`]: 0,
                        [`${this.joystickId}Y`]: 0,
                    };
                    if(this.joystickId == "j1"){
                        this.joystick.style.top = '85%';
                    }
                    // Invia dati di reset
                    /*if(this.joystickId == "j1"){
                        if(altitudineBloccata){
                            joystickData[`${this.joystickId}Y`]= 127;
                        }else{
                            this.joystick.style.top = '85%';
                        }
                    }else{
                        this.joystick.style.top = '50%';      
                    }*/
                    ws.send(JSON.stringify(joystickData));

                    event.preventDefault();
                    return;
                }
            }
        }

        bloccaAltitudine(bloccato){
            if(bloccato){
                this.joystick.style.top = '50%';

                var joystickData = {
                    ["j1Y"]: (127).toFixed(2)
                };

                document.getElementById('input1').value = joystickData["j1Y"];

                ws.send(JSON.stringify(joystickData));
            }else{
                this.joystick.style.top = '85%';
            }
        }
    }

    var j1 = new Joystick(document.getElementById('j1'), 'j1');
    new Joystick(document.getElementById('j2'), 'j2');

    function bloccaJ2Y(){
        altitudineBloccata = !(altitudineBloccata);
        j1.bloccaAltitudine(altitudineBloccata);
    }


    function input1(){
        var inputData = {['input1']: document.getElementById('input1').value};

        ws.send(JSON.stringify(inputData));
    }
    function input2(){
        var inputData = {['input2']: document.getElementById('input2').value};

        ws.send(JSON.stringify(inputData));
    }
    function input3(){
        var inputData = {['input3']: document.getElementById('input3').value};

        ws.send(JSON.stringify(inputData));
    }