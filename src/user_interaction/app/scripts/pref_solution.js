
function create_pref_msg(answer_name, txt, callback_click = undefined) {
    let container = document.createElement("div");
    let h1 = document.createElement("h1");
    h1.innerHTML = txt;
    container.appendChild(h1);
    h1.setAttribute("class", "full_screen");

    // ventana de bloqueo
    let block_w = document.createElement("div");
    block_w.setAttribute("class", "container_block_msg")
    container.appendChild(block_w)

    if (answer_name == "sam") {
        container.setAttribute("class", "option_text");
        container.classList.add("container_msg");
        block_w.setAttribute("value", "sam");
        // se clona el slider
        let container_slider = document.getElementById("left_metric");
        let clone = container_slider.cloneNode(true);
        clone.setAttribute("id", "img_content_msg");
        clone.style.visibility = "";
        container.appendChild(clone);
        // document.getElementById("msg_quest").appendChild(clone);

    }
    if (answer_name == "traditional") {
        container.setAttribute("class", "option_text ");
        container.classList.add("container_msg");
        block_w.setAttribute("value", "traditional");
        const img_p = new Image();
        img_p.onload = () => {
            // do something
        };
        img_p.src = `../img/puntero.png`;
        img_p.setAttribute("class", "img_content_msg");
        img_p.style.width = "230px";
        container.appendChild(img_p);
        // document.getElementById("msg_quest").appendChild(img_p);
    }
    if (answer_name == "ask1") {
        container.setAttribute("class", "ask_text");
    }

    if (callback_click != undefined)
        container.onclick = callback_click;


    return container;
}

function show_option_ask1(answer_name, txt, callback_click = undefined) {
    let c_all = document.getElementById("msg_quest");
    c_all.innerHTML = "";

    let cont_up = document.createElement("div");
    cont_up.setAttribute("id", "container_all_up");
    let cont = document.createElement("div");


    if (answer_name == "sam" || answer_name == "traditional" || answer_name == "ask1") {
        cont.setAttribute("id", "container_full_pref_msg");

        let pref_msg = create_pref_msg(answer_name, txt);
        cont.appendChild(pref_msg);
        cont_up.appendChild(cont)
    }
    if (answer_name == "two_msg") {
        cont.setAttribute("id", "container_all_pref_msg");

        let pref_msg1 = create_pref_msg("sam", "SAM");
        pref_msg1.childNodes[0].setAttribute("id", "")
        let pref_msg2 = create_pref_msg("traditional", "puntero");
        pref_msg2.childNodes[0].setAttribute("id", "")

        cont.appendChild(pref_msg1);
        cont.appendChild(pref_msg2);
        cont_up.appendChild(cont);

        // boton de salida
        let cont_btn = document.createElement("div");
        cont_btn.setAttribute("id", "container_btn_end_experiment")
        let btn_exit = document.createElement("button");
        btn_exit.setAttribute("class", "button-80");
        btn_exit.setAttribute("id", "btn_end_experiment")
        btn_exit.innerHTML = "salida";
        cont_btn.appendChild(btn_exit);
        cont_up.appendChild(cont_btn);
    }
    c_all.appendChild(cont_up);


}



function start_ask1(exp_sam, exp_trad) {
    return new Promise((resolve, reject) => {
        // exp_sam = { id_pareto_front: "optimized-30", num_ind: 10 };
        // exp_trad = { id_pareto_front: "optimized-30", num_ind: 10 };

        resize_win("normal");
        show_one_window("questionnaire");

        // show_option_ask1("sam", "SAM");
        // setTimeout(() => {
        //     simulate_individual(exp_sam.id_pareto_front, exp_sam.num_ind).then(() => {


        //         show_option_ask1("traditional", "puntero");
        //         setTimeout(() => {
        //             simulate_individual(exp_trad.id_pareto_front, exp_trad.num_ind).then(() => {



        //             }).catch((e) => {
        //                 console.error(e);
        //             });
        //         }, 2000);
        //     }).catch((e) => {
        //         console.error(e);
        //     });
        // }, 4000);

        show_option_ask1("two_msg", "");
        setTimeout(() => {
            let btns = document.querySelectorAll("#msg_quest .container_msg")
            for (let i = 0; i < btns.length; i++) {
                btns[i].onclick = (e) => {
                    let is_btn = e.target.getAttribute("value");

                    let data_sim;
                    if (is_btn == "sam") data_sim = exp_sam;
                    if (is_btn == "traditional") data_sim = exp_trad;
                    simulate_individual(data_sim.id_pareto_front, data_sim.num_ind).catch((e) => {
                        console.error(e);
                    });
                }
            }

            // salida btn
            let btn_exit_exp = document.getElementById("btn_end_experiment");
            btn_exit_exp.onclick = () => {

                // se muestra el elemento del video
                show_one_window("intro_video");
                // se guarda el experimento a ejecutar tras terminar el video
                let cont_vid = document.getElementById("container_intro_video");
                cont_vid.setAttribute("ini_experiment", "end_exp");
                load_video("../source/Cuestionario.mp4")

            };

        }, 300);
    });
}


// start_ask1()


// show_option_ask1("traditional", "puntero");
// show_option_ask1("sam", "SAM");
// show_option_ask1("ask1", "Tus soluciones");