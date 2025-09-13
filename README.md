# controller

## 패키지 설명
전체 제어기 부분을 담당하는 코드입니다.

## 코드 실행 방법
터미널에서 아래와 같이 실행합니다:

```
ros2 launch controller controller_launch.xml
```

## 제어기 파라미터 수정 방법
`config` 폴더 내의 두 파일을 수정하면 됩니다:
- `l1_params.yaml`
- `RBC1_pacejka_lookup_table.csv`
