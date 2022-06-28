; hitagimon
; Copyright (c) 2020-2021, Joshua Scoggins
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions are met:
;     * Redistributions of source code must retain the above copyright
;       notice, this list of conditions and the following disclaimer.
;     * Redistributions in binary form must reproduce the above copyright
;       notice, this list of conditions and the following disclaimer in the
;       documentation and/or other materials provided with the distribution.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
; ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
; ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
(deffunction generic-test
             (?fn ?a ?b)
             (loop-for-count (?i ?a ?b) do
                             (printout t ?i ": " (funcall ?fn ?i) crlf)))
(deftemplate single-argument-test
             (slot function
                   (type SYMBOL)
                   (default ?NONE)))
(deffacts single-argument-tests
          (single-argument-test (function sin))
          (single-argument-test (function cos))
          (single-argument-test (function tan))
          )



(defrule generate-single-argument-test-routine
         ?f <- (single-argument-test (function ?fn))
         =>
         (retract ?f)
         (build (format nil
                        "(deffunction indirect-%s-test (?a ?b) (generic-test %s ?a ?b))"
                        ?fn
                        ?fn))
         (build (format nil
                        "(deffunction direct-%s-test (?a ?b) (loop-for-count (?i ?a ?b) do (printout t ?i \": \" (%s ?i) crlf)))"
                        ?fn
                        ?fn)))

(reset)
(run)