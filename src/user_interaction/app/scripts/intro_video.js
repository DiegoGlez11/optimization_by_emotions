
let video_intro = document.getElementById("video_intro");
// video_intro.style.display = "none";

video_intro.addEventListener("ended", () => {

    // datos del inicio del experimento
    let cont_vid = document.getElementById("container_intro_video");
    let data_exp = cont_vid.getAttribute("ini_experiment");
    data_exp = data_exp == null ? "{}" : data_exp;

    // accion de salto
    if (data_exp == "skip") return;
    if (data_exp == "run_questionnaire") {
        run_questionnaire();
        return;
    }
    if (data_exp == "end_exp") {
        //ventana de despedida
        show_one_window("goodbye");
        // pantalla completa
        openFullscreen();
        return;
    }

    // objeto de los datos del experimento
    data_exp = JSON.parse(data_exp);

    if (data_exp.play_intro != undefined) {
        // se elimina la reproducción del video
        delete data_exp.play_intro;
        document.getElementById("container_intro_video").setAttribute("ini_experiment", JSON.stringify(data_exp));
        // reproducción del video del experimento
        let dir_v = get_video_path(false);
        if (dir_v == "") return;
        load_video(dir_v);

    } else {
        // se oculta el video introductorio
        show_one_window("graph");
        start_experiment(data_exp.id_pareto, data_exp.individuals, true, data_exp.extend_search);
    }
});

function play_video(id_pareto, num_inds, play_intro = false, extend_search = undefined) {
    if (id_pareto == undefined || id_pareto == "") {
        alert("No se puede ejecutar el video de introducción. Falta ingresar los datos del frente inicial");
        return;
    }

    // se muestra el elemento del video
    show_one_window("intro_video");

    // se guarda el experimento a ejecutar tras terminar el video
    let cont_vid = document.getElementById("container_intro_video");
    let data_exp = { id_pareto: id_pareto, individuals: num_inds, play_intro: play_intro, extend_search: extend_search };
    cont_vid.setAttribute("ini_experiment", JSON.stringify(data_exp));


    if (play_intro) {
        // dir del video a mostrar
        let dir_vid = get_video_path(true);
        if (dir_vid == "") return;

        load_video(dir_vid);
    } else {
        // se salta la reproducción del video
        let e = new Event("ended");
        video_intro.dispatchEvent(e);
    }

}

function load_video(dir_) {
    // video_intro.remove();

    // se carga el video
    let video_src = document.getElementById("video_intro");
    video_src.setAttribute("src", dir_);
    video_src.setAttribute("type", "video/mp4");
    //  se carga
    video_intro.load()
    video_intro.play();
}

function get_video_path(is_intro) {
    if (is_intro) return "../source/introduccion.mp4";
    else {
        if (EMOTIONS_CAPTURE_MODE == "sam") return "../source/Experimento SAM.mp4";
        if (EMOTIONS_CAPTURE_MODE == "traditional") return "../source/Experimento Grafica.mp4";
    }

    alert("No se encuentra el video");
    return "";
}
