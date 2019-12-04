; ModuleID = 'PPath/c/ifthen.c'
source_filename = "PPath/c/ifthen.c"
target datalayout = "e-m:e-p:32:32-i64:64-n32-S128"
target triple = "leros32-unknown-unknown-elf"

; Function Attrs: noinline nounwind optnone
define dso_local i32 @main() #0 {
entry:
  %retval = alloca i32, align 4
  %x = alloca i32, align 4
  store i32 0, i32* %retval, align 4
  %call = call i32 @rand() #2
  store i32 %call, i32* %x, align 4
  %0 = load i32, i32* %x, align 4
  %cmp = icmp sgt i32 %0, 200
  br i1 %cmp, label %if.then, label %if.end

if.then:                                          ; preds = %entry
  %1 = load i32, i32* %x, align 4
  %inc = add nsw i32 %1, 1
  store i32 %inc, i32* %x, align 4
  br label %if.end

if.end:                                           ; preds = %if.then, %entry
  %2 = load i32, i32* %x, align 4
  ret i32 %2
}

; Function Attrs: nounwind
declare dso_local i32 @rand() #1

attributes #0 = { noinline nounwind optnone "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "min-legal-vector-width"="0" "no-frame-pointer-elim"="true" "no-frame-pointer-elim-non-leaf" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { nounwind "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="true" "no-frame-pointer-elim-non-leaf" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #2 = { nounwind }

!llvm.module.flags = !{!0}
!llvm.ident = !{!1}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{!"clang version 8.0.0 (https://github.com/leros-dev/leros-clang.git ba93d76b060e90d82b2f3f3ccc6488c308790562) (https://github.com/leros-dev/leros-llvm.git d76786d5cf81a937bb119537768b22a68884ca05)"}
