(define (problem p)
  (:domain costtest)
;  (:moduleoptions (trajectoryModuleInit@libps_trajectoryModule.so -s telemax_table.dcscene))
  (:objects car1 - car l1 l2 l3 - location)
  (:init 
      (at car1 l1)
  )
  (:goal (and (at car1 l3) ))
)

