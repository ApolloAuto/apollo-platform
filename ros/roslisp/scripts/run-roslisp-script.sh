#!/usr/bin/env sh
"true";exec /usr/bin/env /usr/bin/sbcl --noinform --end-runtime-options --noprint --no-userinit --disable-debugger --script "$0" "$@"

;; This LISP script can be used to launch LISP scripts that depend on ros functinoality.
;; To use it make the following the first two lines of your script:
;;
;; #!/usr/bin/env sh
;; "true";exec /usr/bin/env rosrun roslisp run-roslisp-script.sh --script "$0" "$@"
;;
;; You can then load asdf systems from ROS (like roslisp) using:
;; (ros-load:load-system "roslisp" "roslisp")
;;
;; This script initializes ros lookup of packages, and wraps SBCL to suppress unwanted output for scripts

(REQUIRE :ASDF)

(labels ((get-roslisp-path ()
           ;; calls rospack to find path to roslisp
           (let ((rospack-process
                   (run-program "rospack" '("find" "roslisp")
                                :search t
                                :output :stream)))
             (when rospack-process
               (unwind-protect
                    (with-open-stream (o (process-output rospack-process))
                      (concatenate 'string (car (loop
                                                  for line := (read-line o nil nil)
                                                  while line
                                                  collect line)) "/load-manifest/"))
                 (process-close rospack-process)))))
         (load-ros-lookup ()
           ;; make sure roslisp is in asdf central registry
           (PUSH (get-roslisp-path) ASDF:*CENTRAL-REGISTRY*)
           ;; load ros-load-manifest, defining e.g. "ros-load:load-system"
           (ASDF:OPERATE 'ASDF:LOAD-OP :ROS-LOAD-MANIFEST :VERBOSE NIL)))
  (load-ros-lookup))



(PUSH :ROSLISP-STANDALONE-EXECUTABLE *FEATURES*)
;; handle conditions
(LABELS ((ROSLISP-DEBUGGER-HOOK (CONDITION ME)
           (DECLARE (IGNORE ME))
           (FLET ((FAILURE-QUIT (&KEY RECKLESSLY-P)
                    (QUIT :UNIX-STATUS 1 :RECKLESSLY-P RECKLESSLY-P)))
             (HANDLER-CASE
                 (PROGN
                   (FORMAT *ERROR-OUTPUT* "~&Roslisp exiting due to condition: ~a~&"
                           CONDITION)
                   (FINISH-OUTPUT *ERROR-OUTPUT*)
                   (FAILURE-QUIT))
               (CONDITION NIL (FAILURE-QUIT :RECKLESSLY-P T))))))
  (UNLESS
      (LET ((V (POSIX-GETENV "ROSLISP_BACKTRACE_ON_ERRORS")))
        (AND (STRINGP V) (> (LENGTH V) 0)))
    (SETQ *INVOKE-DEBUGGER-HOOK* #'ROSLISP-DEBUGGER-HOOK)))

;; load file
(let ((filename (third sb-ext:*posix-argv*)))
  (let ((stream (open filename :direction :input))
        ;; remove artificial arguments
        (sb-ext::*posix-argv* (cddr sb-ext::*posix-argv*)))
    (unwind-protect
         (progn
           ;; ignore shebang line
           (read-line stream)
           (load stream))
      (close stream))))
