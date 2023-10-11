

// +++++++++++++++++++++++++
// ++++ pantalla final +++++
// +++++++++++++++++++++++++

function end_exp(e) {
    if (DATA_CONTROL.active_fatigue) return;
    if (INTERFACE_TEST) return;

    // tipo de termino de exp
    let id_p = e.target.getAttribute("id");
    if (id_p == "a_agree") id_p = "prefered";
    if (id_p == "a_exit") id_p = "exit";

    load_actual_ind().then((actual_ind) => {
        if (actual_ind["id_experiment"] == "" || actual_ind["id_experiment"] == undefined) {
            alert("No hay experimento seleccionado");
            return;
        }

        let data_exp = {};
        data_exp["type_end"] = id_p;
        data_exp["id_pareto_front"] = actual_ind["id_experiment"];
        data_exp["num_ind"] = actual_ind["num_ind"];
        data_exp["storage_position"] = actual_ind["storage_pos"];

        update_experiment(data_exp, actual_ind["storage_pos"]).then(() => {

            // se muestra el elemento del video
            show_one_window("intro_video");
            // se guarda el experimento a ejecutar tras terminar el video
            let cont_vid = document.getElementById("container_intro_video");
            cont_vid.setAttribute("ini_experiment", "skip");

            load_video("../source/Cuestionario.mp4");

            document.getElementById("video_intro").onclick = () => {
                video_intro.pause();
                let video_src = document.getElementById("video_intro");
                video_src.removeAttribute("src");

                video_src.pause();
                video_intro.load();

                // se obtienen los experimentos finalizados
                get_concluded_experiments().then((exp) => {
                    console.log(exp);
                    if (exp.sam.length == 0 || exp.traditional.length == 0) {
                        //ventana de despedida
                        show_one_window("goodbye");
                        // pantalla completa
                        openFullscreen();
                        return;
                    }

                    load_video("../source/Final.mp4");
                    let cont_vid = document.getElementById("container_intro_video");
                    cont_vid.setAttribute("ini_experiment", "run_questionnaire");

                }).catch((e) => {
                    console.error(e);
                });
            }

        }).catch((e) => {
            console.error(e);
        });
    }).catch((e) => {
        console.error(e);
    });
};


function run_questionnaire() {
    document.getElementById("video_intro").onclick = undefined;

    // se obtienen los experimentos finalizados
    get_concluded_experiments().then((exp) => {

        // si se realizan los experimentos
        if (exp.sam.length > 0 && exp.traditional.length > 0) {
            // se usa el último realizado
            let exp_sam = exp.sam.pop();
            let exp_trad = exp.traditional.pop();

            start_ask1(exp_sam, exp_trad).then(() => {


            });
        }
        // else {

        //     //ventana de despedida
        //     show_one_window("goodbye");
        //     // pantalla completa
        //     openFullscreen();
        // }
    }).catch((e) => {
        console.error(e);
    });
}


// +++++++++++++++ botones de fin del experimento +++++++++++++
document.getElementById("a_exit").onclick = end_exp;

// +++++ conforme con la solución ++++++++
document.getElementById("a_agree").onclick = end_exp;


// ++++++++++ se han recorrido todos los frentes ++++++++++++++
document.getElementById("btn_last_front").onclick = end_exp;

//++++++++++++ ventana de terminación del experimento++++++++++

// por fatiga se termina el exp
document.getElementById("btn_si").onclick = end_exp;

