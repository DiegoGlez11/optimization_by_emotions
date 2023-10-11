

// let server = udp_gram.createSocket('udp4');
// const PRE_RUN_TIME = 2000;
// let buffer = "";
// let label = "prerun,0,";
// let name = null;

let videoStarted;
let exitacion = null;
let placer = null;
let video = $('#videoEmo');
let count = $('#count');
let slider = $('#AffectiveSlider');
let rateBtn = $('#rateBtn');
let form = $('#nameForm');

let name_histo_basal;
let name_histo_emotion;
//id del experimento
let num_experiment;
//id de las señales EEG
let id_eeg_signal;
//conteo del experimento
let current = 0;
//detiene el proceso dado en milisegundos
const timer = (ms) => new Promise(resolve => setTimeout(resolve, ms));


let videos = [
    // { vid: 'resources/videoRobot/01_austero_editado.mp4' },
    { vid: 'resources/videoRobot/1_editado.mp4' },
    // { vid: 'resources/videoRobot/02_austero_editado.mp4' },
    { vid: 'resources/videoRobot/2_editado.mp4' },
    // { vid: 'resources/videoRobot/03_austero_editado.mp4' },
    { vid: 'resources/videoRobot/3_editado.mp4' },
    // { vid: 'resources/videoRobot/04_austero_editado.mp4' },
    { vid: 'resources/videoRobot/4_editado.mp4' },
    // { vid: 'resources/videoRobot/05_austero_editado.mp4' },
    { vid: 'resources/videoRobot/5_editado.mp4' },
    // { vid: 'resources/videoRobot/06_austero_editado.mp4' },
    { vid: 'resources/videoRobot/6_editado.mp4' },

    { vid: 'resources/video/1.mp4' },//A Fine Frenzy - Almost Lover Official Video
    { vid: 'resources/video/3.mp4' },//Alphabeat - Fascination
    { vid: 'resources/video/2.mp4' },//Bishop Allen - Butterfly Nets
    { vid: 'resources/video/4.mp4' },//Breakdown - Jack Johnson
    { vid: 'resources/video/5.mp4' },//Emiliana Torrini - Jungle Drum (Official Video)
    { vid: 'resources/video/6.mp4' },//Im From Barcelona Were From Barcelona
    { vid: 'resources/video/7.mp4' },//James Blunt - Goodbye My Lover [OFFICIAL VIDEO]
    { vid: 'resources/video/8.mp4' },//kings of convenience - weight of my words
    { vid: 'resources/video/9.mp4' },//LCD Soundsystem - New York, I Love You But Youre Bringing Me Down
    { vid: 'resources/video/10.mp4' },//Mad World - Gary Jules
    { vid: 'resources/video/11.mp4' },//Marilyn Manson - The Beautiful People (Official Video)
    { vid: 'resources/video/12.mp4' },//METALLICA st anger (HIGH QUALITY)
    { vid: 'resources/video/13.mp4' },//MIKA - Love Today (Official Video)
    { vid: 'resources/video/14.mp4' },//MORTEMIA - The One I Once Was (Official)
    { vid: 'resources/video/15.mp4' },//Old Crow Medicine Show - Caroline [Official Music Video]
    { vid: 'resources/video/16.mp4' },//OREN LAVIE  Her Morning Elegance 
    { vid: 'resources/video/17.mp4' },//Parkway Drive - Smoke Em If Ya Got Em
    { vid: 'resources/video/18.mp4' },//Porcupine Tree - Normal Peaceville Records
    { vid: 'resources/video/19.mp4' },//STIGMATA - В ОТРАЖЕНИИ ГЛАЗ (FAN VIDEO, 2010)
    { vid: 'resources/video/20.mp4' },//The Go! Team - Huddle Formation
];




//------ se ejecuta cuando se cargan los videos ------
document.getElementById('videoEmo').addEventListener('loadedmetadata', onStartVideo, false);
function onStartVideo(e) {

    $("#rateBtn").prop("disabled", false);
}


//------ cuando se ingresa el nombre de usuario ------
let user_input = document.getElementById("name_user");
user_input.addEventListener("keyup", () => {
    // if (e.key != "Enter") return;

    if (user_input.value.length > 0) {
        $("#startEmoDetect").prop("disabled", false);
    } else {
        $("#startEmoDetect").prop("disabled", true);
    }
});

//-------retorna el npmbre del usuario
function get_user() {
    let name = document.getElementById("name_user").value;
    return name.trim();
}

//-------cada vez que se le da click al boton de iniciar
let btn_user = document.getElementById("startEmoDetect");
btn_user.addEventListener("click", () => {
    let dir = `${root_dir}/${get_user()}/protocol`;
    let dir_h = dir + "/history";

    //se crean la carpetas del nuevo usuario
    fs.mkdirSync(dir_h, { recursive: true });

    // calculamos el id del experimento
    create_id("history_basal_", dir_h).then((res) => {
        name_histo_basal = `history_basal_${res.num}.txt`;
        name_histo_emotion = `history_emotion_evaluation_${res.num}.txt`;

        save_object([], name_histo_basal, dir_h);
        save_object([], name_histo_emotion, dir_h);
        num_experiment = res.num;

        //guardamos el id
        save_object({ history_basal: name_histo_basal, history_emotion_evaluation: name_histo_emotion }, "last_experiment.txt", dir_h);

        //se inicia el protocolo
        start();
    });
});


//------ se ejecuta cada vez que termina un video musical------
let video_emo = document.getElementById('videoEmo');
video_emo.addEventListener('ended', onStopVideo, false);
function onStopVideo(e) {

    //se cambia el texto de la valencia y el arousal
    let src = document.getElementById("videoEmo").src;

    // if (src.search("Robot") >= 0) {
    //     change_emo_slider("Insatisfecho - Satisfecho", "Aburrido - Emocionado");
    // } else {
    //     change_emo_slider("Triste - Feliz", "Aburrido - Emocionado")
    // }
    change_emo_slider("NO placentero - Placentero", "Aburrido - Emocionado")

    //se establece el valor de los affective slider a su valor predeterminado
    document.getElementById("arousal").value = 0.5;
    document.getElementById("valence").value = 0.5;

    //detenemos el flujo EEG
    stop_eeg().then(() => {
        //se muestra el affective slaider
        slider.show();
        rateBtn.show();
    }).catch(() => {
        alert("Error al detener el flujo EEG del video");
    });
}

// cuando acaba del video de introducción
let video_intro = document.getElementById("video_intro");
video_intro.addEventListener("ended", () => {
    $("#video_intro").hide();
    form.show();
}, false);


//se inicia la ejecución al cargar la página
window.onload = function () {
    // slider.show();
    // rateBtn.show();
    form.hide();
    slider.hide();
    rateBtn.hide();
    video.hide();

    //se oculta el contador
    document.getElementById("count").innerHTML = "0";
    count.hide();

    stop_eeg();
}

//------ Inicia una transición del protocolo ------
// la cual consiste en 1:número de video 2:cruz para centrar la mira 3:reproducir video
async function start() {
    video.hide(); // se oculta el video
    form.hide(); // se oculta el campo de usuario
    slider.hide(); // se oculta el affective slider
    rateBtn.hide(); // se oculta el boton de evaluación


    if (current >= videos.length) {
        count.text("Fin");
        count.show();

        await timer(5000);

        return;
    }

    //iniciamos la captura del flujo EEG del estado basal
    play_eeg("protocol_basal", current, true, true).then(() => {
        count.text(`${current + 1}/${videos.length}`);
        count.show();
        let duration_num = 2000;

        setTimeout(() => {

            //cruz para centrar la mirada
            // count.text('✚');

            //duración del estado basal
            // let duration_basal_state = 5000; //milisegundos
            // setTimeout(() => {
            //detenemos el flujo EEG del estado basal
            stop_eeg().then((res) => {
                let obj = { num_video: current, id_eeg_signal: res.id_eeg_signal, src: videos[current].vid };
                //actualizamos el historial
                add_history(obj, name_histo_basal);
                id_eeg_signal_g = res.id_eeg_signal;

                let aux_time = 100;
                setTimeout(() => {

                    //se inicia el flujo EEG del video
                    play_eeg("protocol_video", current, true, true).then(() => {
                        //se oculta la cruz
                        count.hide();

                        //se muestra el siguiente video
                        document.getElementById("videoEmo").src = videos[current].vid;
                        video.show();


                    }).catch((d) => {
                        console.error(d);
                        alert("Error al iniciar la captura EEG del video");
                        // location.reload();
                    });
                }, aux_time);
            }).catch((d) => {
                console.error(d);
                alert("Error al detener la captura EEG del estado basal");
                // location.reload();
            });
            // }, duration_basal_state);
        }, duration_num);
    }).catch((e) => {
        console.error(e);
        alert("Error al iniciar la captura EEG del estado basal");
    });
}


let id_eeg_signal_g;
//se ejecuta cada vez que el usuario da una evelacuación emocional 
//a un video por medio del botón calificar
let btn = document.getElementById("rateBtn");
btn.addEventListener("click", () => {
    //ocultamos el boton
    // $("#rateBtn").prop("disabled", true);

    //se obtiene el valor de las evaluaciones emocionales
    exitacion = document.getElementById("arousal").value;
    placer = document.getElementById("valence").value;

    let evalu = { valence: placer, arousal: exitacion, id_eeg_signal: id_eeg_signal_g, num_video: current, src: videos[current].vid }

    //guardamos la evaluación
    add_history(evalu, name_histo_emotion);
    current++;

    start();
});


function add_history(obj, file_name) {
    return new Promise((resolve, reject) => {
        let dir = `${root_dir}/${get_user()}/protocol/history`;

        load_object(`${dir}/${file_name}`).then((histo) => {

            if (histo == undefined) {
                console.error("Error al cargar el historial", file_name);
                reject();
                // resolve();
                return;
            }

            histo.push(obj);

            //se guarda el historial actualizado
            save_object(histo, file_name, dir).then(() => {
                resolve();
            }).catch((e) => {
                reject(e);
            });
        }).catch((f) => {
            reject(f)
        });
    });
}



function change_emo_slider(valence_name, arousal_name) {
    let arousal_part = `
    <div class="container_metric">
        <object class="img_metric" type="image/svg+xml" data="resources/images/AS_sleepy.svg"></object>
        <div class="russell_metric">
            <input type="range" min="0" max="1" value="0.5" id="arousal" step="0.01">
            <div class="triangle left"></div>
            <div class="triangle right"></div>
            <div id="color_arousal"></div>
            <label for="arousal" class="label_indicator">${arousal_name}</label>
        </div>
        <object class="img_metric" type="image/svg+xml" data="resources/images/AS_wideawake.svg"></object>
    </div>`;

    let pleasure_part = `
    <div class="container_metric">
        <object class="img_metric" type="image/svg+xml" data="resources/images/AS_unhappy.svg"></object>

        <div class="russell_metric">
            <input type="range" min="0" max="1" value="0.5" id="valence" step="0.01">
            <div class="triangle left"></div>
            <div class="triangle right"></div>
            <div id="color_valance"></div>
            <label for="arousal" class="label_indicator">${valence_name}</label>
        </div>
        <object class="img_metric" type="image/svg+xml" data="resources/images/AS_happy.svg"></object>
    </div>`;

    document.getElementById('AffectiveSlider').innerHTML = pleasure_part + arousal_part;

}

function load_object(path_obj) {
    return new Promise((resolve, reject) => {
        fs.readFile(path_obj, 'utf8', (err, data) => {
            if (err) {
                resolve();
            } else {
                let obj = {};
                if (data.length > 0) obj = JSON.parse(data);
                resolve(obj);
            }
        });
    });
}


function save_object(obj, name_file, dir_) {
    return new Promise((res, rej) => {
        save_data_string(JSON.stringify(obj), name_file, dir_).then((e) => {
            res(e);
        }).catch((g) => {
            rej(g);
        });
    });
}


// play_eeg("test", 0, true, false).then(() => { console.log(".."); }).catch(() => { console.log("error", e); });



// //inicia la conexión con el servidor
// function startServer() {
//     // En caso de error
//     server.on('error', (err) => {
//         console.error("Error:", err);
//         server.close();
//         process.exit();
//     })

//     // En caso de que 
//     server.on('listening', function () {
//         var address = server.address();
//         var port = address.port;
//         var family = address.family;
//         var ipaddr = address.address;
//         console.log('Server is listening at port ' + port);
//         console.log('Server ip :' + ipaddr);
//         console.log('Server is IP4/IP6 : ' + family);
//     });


//     // Se llama cuando llega un nuevo datagram msg
//     server.on('message', function (msg, info) {
//         if (name != null || name != 'null') {
//             var msgLength = msg.toString().length
//             buffer = (label + msg.toString().substring(22, msgLength - 4));
//             buffer += "\n";
//             fs.appendFile("app/data/" + name + ".csv", buffer, (err) => {
//                 if (err) {
//                     console.log("error: " + err)
//                 }
//                 else {
//                     console.log("saved data eeg file");
//                 }
//             });
//         }
//     })
//     server.bind(12345);
// }


