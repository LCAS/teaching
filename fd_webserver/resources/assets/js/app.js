import bootstrap from 'bootstrap-sass';
import CodeMirror from 'codemirror';
import fileSaver from 'file-saver';
import jQuery from 'jquery';
import commonlisp from 'codemirror/mode/commonlisp/commonlisp.js';

window._ = require('lodash');
window.$ = window.jQuery = jQuery;
window.CodeMirror = CodeMirror;

window.onload = function() {

    var domain_editor = CodeMirror.fromTextArea(document.getElementById("domain"), {
        lineNumbers: true,
        styleActiveLine: true,
        matchBrackets: true,
        theme: "eclipse",
        mode: "commonlisp" //"text/x-common-lisp"
    });

    var problem_editor = CodeMirror.fromTextArea(document.getElementById("problem"), {
        lineNumbers: true,
        styleActiveLine: true,
        matchBrackets: true,
        theme: "eclipse",
        mode: "commonlisp" //"text/x-common-lisp"
    });

    $("#ddm-domain li a").click(function() {
        $.get("static/domains/" + $(this).text() + ".pddl", function(data) {
            domain_editor.setValue(data)
        });
        console.log("Selected Option:" + $(this).text());
    });

    $('#download_domain').on('click', function() {
        var blob = new Blob([domain_editor.getValue()], {
            type: "text/plain;charset=ascii"
        });
        fileSaver.saveAs(blob, "domain.pddl");
    });

    $("#ddm-problem li a").click(function() {
        $.get("static/problems/" + $(this).text() + ".pddl", function(data) {
            problem_editor.setValue(data)
        });
        console.log("Selected Option:" + $(this).text());
    });

    $('#planit').on('click', function() {
        var $btn = $(this).button('loading');
        $.post(document.URL, {
                domain: domain_editor.getValue(),
                problem: problem_editor.getValue()
            }, function(data) {
                $('#sout').val(data.sout)
                $('#plan').val(data.plan)
            })
            .always(function() {
                $btn.button('reset');
            })
            .fail(function() {
                alert("An error occurred that most likely caused by the planner timing out. This means the problem it was trying to solve was too hard, and your domain file should probably be redesigned to reduce the number of applicable actions for a given state (e.g. by using stronger typing).");
            });
    });
};
